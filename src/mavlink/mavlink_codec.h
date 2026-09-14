#pragma once
// ============================================================================
// SkyGCS  MAVLink v1/v2 编解码器
//   - 帧解析 (增量流式, 支持 v1/v2 混用, 载荷截断, 扩展字段)
//   - 帧封装 (v2, 基础载荷, CRC-16/MCRF4XX + crc_extra)
//   - 消息 打包/解包 (基于生成的消息定义表, 字段序与官方 c_library_v2 一致)
// 不依赖 Qt。
// ============================================================================
#include <cstdint>
#include <cstddef>
#include <vector>

#include "mavlink_types.h"
#include "mavlink_generated.h"

namespace skygcs {

class MavlinkCodec {
public:
    // 本端身份 (作为 GCS)
    void setLocalIds(uint8_t sysid, uint8_t compid);
    uint8_t localSysid() const { return sysid_; }
    uint8_t localCompid() const { return compid_; }

    // ------------------------------------------------------------------
    // CRC (公开, 便于测试)
    // ------------------------------------------------------------------
    static uint16_t crcAccumulate(uint8_t data, uint16_t crc);
    static uint16_t crcOfBytes(const uint8_t* data, size_t len, uint16_t init = 0xFFFF);

    // ------------------------------------------------------------------
    // 编码
    // ------------------------------------------------------------------
    // 将消息打包为完整 v2 帧 (含 STX); 非 const: 内部推进 seq
    std::vector<uint8_t> encodeFrame(const MavMessage& msg);

    // 将结构体指针按消息定义打包为 MavMessage (payload 为 minLen)
    static bool pack(const MsgDef& def, const void* structPtr, MavMessage& out);

    // ------------------------------------------------------------------
    // 解码 (增量流式)
    // ------------------------------------------------------------------
    // 追加原始字节流, 每次调用尝试解析缓冲区头部的一条完整消息
    // 返回 Complete 时 out 为解析结果, 已消费的字节从缓冲区移除
    ParseStatus decodeNext(const uint8_t* data, size_t len, MavMessage& out);
    void reset();

    // 便捷: 按消息定义将载荷解包为结构体 (允许载荷短于 minLen, 缺失字节补 0)
    static bool unpack(const MsgDef& def, const uint8_t* payload, size_t payloadLen, void* structPtr);

    // 根据消息 ID 查找定义
    static const MsgDef* findDef(uint32_t msgid);

private:
    uint8_t sysid_ = 255;   // GCS 常用 255
    uint8_t compid_ = 190;  // MAV_COMP_ID_GCS = 190
    uint8_t seq_ = 0;

    std::vector<uint8_t> buf_;
};

} // namespace skygcs
