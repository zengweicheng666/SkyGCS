// ============================================================================
// SkyGCS  MAVLink v1/v2 编解码器实现
// ============================================================================
#include "mavlink_codec.h"

#include <cstring>

namespace skygcs {

namespace {
constexpr uint8_t  STX_V1 = 0xFE;
constexpr uint8_t  STX_V2 = 0xFD;
constexpr size_t   MAX_PAYLOAD = 255;
constexpr uint16_t CRC_INIT = 0xFFFF;
} // namespace

// ---------------------------------------------------------------------------
// CRC-16/MCRF4XX —— 与 mavlink C 库 crc_accumulate 逐位一致
// ---------------------------------------------------------------------------
uint16_t MavlinkCodec::crcAccumulate(uint8_t data, uint16_t crc)
{
    uint8_t tmp = static_cast<uint8_t>(data ^ (crc & 0xFF));
    tmp = static_cast<uint8_t>(tmp ^ (tmp << 4));
    return static_cast<uint16_t>((crc >> 8) ^ (static_cast<uint16_t>(tmp) << 8)
                                 ^ (static_cast<uint16_t>(tmp) << 3) ^ (tmp >> 4));
}

uint16_t MavlinkCodec::crcOfBytes(const uint8_t* data, size_t len, uint16_t init)
{
    uint16_t crc = init;
    for (size_t i = 0; i < len; ++i)
        crc = crcAccumulate(data[i], crc);
    return crc;
}

// ---------------------------------------------------------------------------
// 消息表查找
// ---------------------------------------------------------------------------
const MsgDef* MavlinkCodec::findDef(uint32_t msgid)
{
    for (size_t i = 0; i < kMsgDefCount; ++i) {
        if (kMsgDefs[i].id == msgid)
            return &kMsgDefs[i];
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// 打包/解包 (基于生成的定义表; 字段序 == 类型长度降序, 故 C 偏移 == wire 偏移)
// ---------------------------------------------------------------------------
bool MavlinkCodec::pack(const MsgDef& def, const void* structPtr, MavMessage& out)
{
    if (!structPtr)
        return false;
    const uint8_t* base = static_cast<const uint8_t*>(structPtr);
    out.msgid = def.id;
    out.payload.assign(def.minLen, 0);
    for (uint16_t i = 0; i < def.fieldCount; ++i) {
        const FieldDef& f = def.fields[i];
        const size_t count = f.count ? f.count : 1;   // 0 表示标量
        const size_t bytes = static_cast<size_t>(f.size) * count;
        std::memcpy(out.payload.data() + f.wireOff, base + f.wireOff, bytes);
    }
    return true;
}

bool MavlinkCodec::unpack(const MsgDef& def, const uint8_t* payload,
                          size_t payloadLen, void* structPtr)
{
    if (!payload || !structPtr)
        return false;
    uint8_t* base = static_cast<uint8_t*>(structPtr);
    std::memset(base, 0, def.minLen);
    for (uint16_t i = 0; i < def.fieldCount; ++i) {
        const FieldDef& f = def.fields[i];
        const size_t count = f.count ? f.count : 1;   // 0 表示标量
        const size_t bytes = static_cast<size_t>(f.size) * count;
        size_t avail = (payloadLen >= f.wireOff) ? payloadLen - f.wireOff : 0;
        std::memcpy(base + f.wireOff, payload + f.wireOff, bytes < avail ? bytes : avail);
    }
    return true;
}

// ---------------------------------------------------------------------------
// 编码: 打包为 v2 帧
// ---------------------------------------------------------------------------
std::vector<uint8_t> MavlinkCodec::encodeFrame(const MavMessage& msg)
{
    std::vector<uint8_t> frame;
    frame.reserve(12 + msg.payload.size() + 2);

    // v2 帧头
    frame.push_back(STX_V2);
    frame.push_back(static_cast<uint8_t>(msg.payload.size()));   // len
    frame.push_back(0);                                          // incompat_flags
    frame.push_back(0);                                          // compat_flags
    frame.push_back(seq_++);                                     // seq
    frame.push_back(msg.sysid);
    frame.push_back(msg.compid);
    // msgid 3 字节小端
    frame.push_back(static_cast<uint8_t>(msg.msgid & 0xFF));
    frame.push_back(static_cast<uint8_t>((msg.msgid >> 8) & 0xFF));
    frame.push_back(static_cast<uint8_t>((msg.msgid >> 16) & 0xFF));
    // 载荷
    frame.insert(frame.end(), msg.payload.begin(), msg.payload.end());

    // CRC: 从 len 字节到载荷末尾 + crc_extra
    const MsgDef* def = findDef(msg.msgid);
    uint8_t crcExtra = def ? def->crcExtra : 0;
    uint16_t crc = CRC_INIT;
    for (size_t i = 1; i < frame.size(); ++i)   // 跳过 STX
        crc = crcAccumulate(frame[i], crc);
    crc = crcAccumulate(crcExtra, crc);
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

// ---------------------------------------------------------------------------
// 解码: 增量流式解析
// ---------------------------------------------------------------------------
void MavlinkCodec::reset() { buf_.clear(); }

void MavlinkCodec::setLocalIds(uint8_t sysid, uint8_t compid)
{
    sysid_ = sysid;
    compid_ = compid;
}

ParseStatus MavlinkCodec::decodeNext(const uint8_t* data, size_t len, MavMessage& out)
{
    buf_.insert(buf_.end(), data, data + len);

    // 扫描缓冲区寻找合法帧
    size_t pos = 0;
    while (pos + 2 <= buf_.size()) {
        const uint8_t stx = buf_[pos];

        if (stx == STX_V2) {
            // 需要完整头部 (10 字节) 才知道 len
            if (pos + 10 > buf_.size())
                break;
            size_t payloadLen = buf_[pos + 1];
            const uint8_t incompat = buf_[pos + 2];
            if (incompat & 0x01) {
                // MAVLink 2 带签名帧: 本地面站不支持签名, 跳过
                pos += 1;
                continue;
            }
            size_t total = 10 + payloadLen + 2;   // 头部+载荷+CRC
            if (pos + total > buf_.size())
                break;

            // CRC 校验 (覆盖 len..载荷, 不含 STX)
            uint16_t crc = CRC_INIT;
            for (size_t i = 1; i < 10 + payloadLen; ++i)
                crc = crcAccumulate(buf_[pos + i], crc);
            const uint32_t msgid = static_cast<uint32_t>(buf_[pos + 7])
                                 | (static_cast<uint32_t>(buf_[pos + 8]) << 8)
                                 | (static_cast<uint32_t>(buf_[pos + 9]) << 16);
            const MsgDef* def = findDef(msgid);
            const uint8_t crcExtra = def ? def->crcExtra : 0;
            crc = crcAccumulate(crcExtra, crc);

            const uint16_t crcRx = static_cast<uint16_t>(buf_[pos + 10 + payloadLen])
                                 | (static_cast<uint16_t>(buf_[pos + 10 + payloadLen + 1]) << 8);
            if (crcRx != crc) {
                pos += 1;   // CRC 失败: 滑动一个字节继续找帧
                continue;
            }

            out.msgid = msgid;
            out.sysid = buf_[pos + 5];
            out.compid = buf_[pos + 6];
            out.seq = buf_[pos + 4];
            out.isV2 = true;
            out.payload.assign(buf_.begin() + pos + 10, buf_.begin() + pos + 10 + payloadLen);
            buf_.erase(buf_.begin(), buf_.begin() + pos + total);
            return ParseStatus::Complete;
        }
        else if (stx == STX_V1) {
            if (pos + 6 > buf_.size())
                break;
            size_t payloadLen = buf_[pos + 1];
            size_t total = 6 + payloadLen + 2;
            if (pos + total > buf_.size())
                break;

            uint16_t crc = CRC_INIT;
            for (size_t i = 1; i < 6 + payloadLen; ++i)
                crc = crcAccumulate(buf_[pos + i], crc);
            const uint32_t msgid = buf_[pos + 5];
            const MsgDef* def = findDef(msgid);
            const uint8_t crcExtra = def ? def->crcExtra : 0;
            crc = crcAccumulate(crcExtra, crc);

            const uint16_t crcRx = static_cast<uint16_t>(buf_[pos + 6 + payloadLen])
                                 | (static_cast<uint16_t>(buf_[pos + 6 + payloadLen + 1]) << 8);
            if (crcRx != crc) {
                pos += 1;
                continue;
            }

            out.msgid = msgid;
            out.sysid = buf_[pos + 3];
            out.compid = buf_[pos + 4];
            out.seq = buf_[pos + 2];
            out.isV2 = false;
            out.payload.assign(buf_.begin() + pos + 6, buf_.begin() + pos + 6 + payloadLen);
            buf_.erase(buf_.begin(), buf_.begin() + pos + total);
            return ParseStatus::Complete;
        }
        else {
            pos += 1;   // 非法字节, 跳过
        }
    }

    // 清理已扫描但无效的前缀 (防止缓冲区无限增长)
    if (pos > 0)
        buf_.erase(buf_.begin(), buf_.begin() + pos);
    return ParseStatus::NeedMore;
}

} // namespace skygcs
