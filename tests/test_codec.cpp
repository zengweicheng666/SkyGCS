// ============================================================================
// SkyGCS  MAVLink 编解码交叉验证测试 (无 Qt 依赖, 可独立编译运行)
//
// 参考帧来源: pymavlink v20 dialect (MAVLink 2) —— 独立第三方实现
//   编码参数: srcSystem=255, srcComponent=190, seq=0 (每条用独立 codec)
//   注1: COMMAND_LONG 的 confirmation=0 被 pymavlink 按 MAVLink2 规则截断,
//        故不参与逐字节比对 (我们的完整帧同样合法, 已单独做解包校验)
//   注2: pymavlink v20 方言的 SYS_STATUS 为旧定义(8×u16, 29B);
//        现行 c_library_v2 为 9×u16 (31B 基础 + 12B 扩展 = 43B)。
//        本工程以官方现行头文件为准, SYS_STATUS 改用往返+官方 CRC 校验。
// ============================================================================
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "../src/mavlink/mavlink_codec.h"
#include "../src/mavlink/mavlink_messages.h"

using namespace skygcs;

static int g_fail = 0;

static void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        ++g_fail;
}

static std::string toHex(const std::vector<uint8_t>& v)
{
    static const char* H = "0123456789ABCDEF";
    std::string s;
    for (uint8_t b : v) {
        s += H[b >> 4];
        s += H[b & 0xF];
    }
    return s;
}

// 参考帧 (pymavlink v20, MAVLink2, sys=255 comp=190 seq=0)
static const char* kRefAttitude =
    "FD1C000000FFBE1E000040E20100CDCCCC3DCDCC4CBE9A99993E0AD7233C0AD7A3BC8FC2F53C76CE";
static const char* kRefHeartbeat =
    "FD09000000FFBE00000000000304020C810403F2A1";
static const char* kRefSysStatus =
    "FD1D000000FFBE010000FFFFFFFFFFFFFFFFF0FFFFFF4001A0415203000000000000000000002DEBBA";
static const char* kRefGlobalPositionInt =
    "FD1C000000FFBE210000E8030000E0ED9B0E5032030730750000881300000A00ECFF1E00282359A1";

static bool hexEq(const std::vector<uint8_t>& v, const char* hex)
{
    const std::string got = toHex(v);
    return got == hex;
}

int main()
{
    MavlinkCodec codec;
    codec.setLocalIds(255, 190);

    // ---- 1) ATTITUDE 逐字节比对 ----
    {
        AttitudeMsg a{};
        a.time_boot_ms = 123456;
        a.roll = 0.1f; a.pitch = -0.2f; a.yaw = 0.3f;
        a.rollspeed = 0.01f; a.pitchspeed = -0.02f; a.yawspeed = 0.03f;
        MavMessage msg;
        msg.sysid = 255;   // 与参考帧一致 (GCS 身份)
        msg.compid = 190;
        check(MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_ATTITUDE), &a, msg),
              "pack ATTITUDE");
        std::vector<uint8_t> frame = codec.encodeFrame(msg);
        check(hexEq(frame, kRefAttitude), "ATTITUDE 帧 == pymavlink 参考帧");
    }

    // ---- 2) HEARTBEAT 逐字节比对 ----
    {
        HeartbeatMsg h{};
        h.type = 2; h.autopilot = 12; h.base_mode = 129;
        h.custom_mode = mav::px4CustomModeEncode(mav::PX4_MODE_AUTO, mav::PX4_AUTO_LOITER);
        h.system_status = 4; h.mavlink_version = 3;
        MavMessage msg;
        msg.sysid = 255; msg.compid = 190;
        MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_HEARTBEAT), &h, msg);
        MavlinkCodec enc;   // 独立 codec → seq=0
        check(hexEq(enc.encodeFrame(msg), kRefHeartbeat), "HEARTBEAT 帧 == pymavlink 参考帧");
    }

    // ---- 3) SYS_STATUS: 官方现行布局 (31B 基础/43B 完整, crc_extra=124)
    //        往返校验 + 帧长校验 (pymavlink v20 为旧布局, 不做逐字节比对)
    {
        SysStatusMsg s{};
        s.onboard_control_sensors_present = 0xFFFFFFFF;
        s.onboard_control_sensors_enabled = 0xFFFFFFFF;
        s.onboard_control_sensors_health = 0xFFFFFFF0;
        s.load = 320; s.voltage_battery = 16800; s.current_battery = 850;
        s.battery_remaining = 45;
        MavMessage msg;
        msg.sysid = 255; msg.compid = 190;
        MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_SYS_STATUS), &s, msg);
        std::vector<uint8_t> frame = codec.encodeFrame(msg);
        check(frame.size() == 12 + 31, "SYS_STATUS 帧 43B (31B 基础载荷)");

        MavlinkCodec rx;
        MavMessage out;
        ParseStatus st = rx.decodeNext(frame.data(), frame.size(), out);
        SysStatusMsg back{};
        bool ok = st == ParseStatus::Complete && out.msgid == 1
               && MavlinkCodec::unpack(*MavlinkCodec::findDef(MAV_MSG_ID_SYS_STATUS),
                                       out.data(), out.size(), &back);
        check(ok && back.load == 320 && back.voltage_battery == 16800
                  && back.battery_remaining == 45
                  && MavlinkCodec::findDef(1)->crcExtra == 124
                  && MavlinkCodec::findDef(1)->minLen == 31
                  && MavlinkCodec::findDef(1)->fullLen == 43,
              "SYS_STATUS 往返一致 + 官方 crc=124 len=31/43");
    }

    // ---- 4) GLOBAL_POSITION_INT 逐字节比对 ----
    {
        GlobalPositionIntMsg g{};
        g.time_boot_ms = 1000;
        g.lat = 245100000; g.lon = 117650000;
        g.alt = 30000; g.relative_alt = 5000;
        g.vx = 10; g.vy = -20; g.vz = 30;
        g.hdg = 9000;
        MavMessage msg;
        msg.sysid = 255; msg.compid = 190;
        MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_GLOBAL_POSITION_INT), &g, msg);
        MavlinkCodec enc;   // 独立 codec → seq=0
        check(hexEq(enc.encodeFrame(msg), kRefGlobalPositionInt),
              "GLOBAL_POSITION_INT 帧 == pymavlink 参考帧");
    }

    // ---- 5) 解码参考帧 (模拟对端收包路径) ----
    {
        const char* refs[] = { kRefAttitude, kRefHeartbeat, kRefSysStatus, kRefGlobalPositionInt };
        const uint32_t ids[] = { 30, 0, 1, 33 };
        MavlinkCodec rx;
        bool allOk = true;
        for (int i = 0; i < 4; ++i) {
            MavMessage msg;
            // 转 hex → 字节
            std::vector<uint8_t> raw;
            for (size_t j = 0; j < std::strlen(refs[i]); j += 2) {
                unsigned v;
                std::sscanf(refs[i] + j, "%2x", &v);
                raw.push_back(static_cast<uint8_t>(v));
            }
            ParseStatus st = rx.decodeNext(raw.data(), raw.size(), msg);
            if (st != ParseStatus::Complete || msg.msgid != ids[i])
                allOk = false;
        }
        check(allOk, "解码 pymavlink 参考帧 (msgid 全部正确)");
    }

    // ---- 6) 自编码→自解码 往返 (含 COMMAND_LONG, 33B 完整帧) ----
    {
        CommandLongMsg c{};
        c.param1 = 1.0f; c.command = mav::CMD_COMPONENT_ARM_DISARM;
        c.target_system = 1; c.target_component = 1; c.confirmation = 0;
        MavMessage msg;
        MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_COMMAND_LONG), &c, msg);
        std::vector<uint8_t> frame = codec.encodeFrame(msg);
        check(frame.size() == 12 + 33, "COMMAND_LONG 完整帧 45B (33B 载荷)");

        MavlinkCodec rx;
        MavMessage out;
        ParseStatus st = rx.decodeNext(frame.data(), frame.size(), out);
        CommandLongMsg back{};
        bool ok = st == ParseStatus::Complete && out.msgid == 76
               && MavlinkCodec::unpack(*MavlinkCodec::findDef(MAV_MSG_ID_COMMAND_LONG),
                                       out.data(), out.size(), &back);
        check(ok && back.command == 400 && back.param1 == 1.0f
                  && back.target_system == 1 && back.confirmation == 0,
              "COMMAND_LONG 往返一致");
    }

    // ---- 7) 流式解析: 两帧粘连 + 头部噪声 ----
    {
        MavlinkCodec rx;
        // 噪声字节 + HEARTBEAT帧 + ATTITUDE帧 粘连
        std::vector<uint8_t> stream = { 0x00, 0xAA, 0xFF };
        for (size_t j = 0; j < std::strlen(kRefHeartbeat); j += 2) {
            unsigned v; std::sscanf(kRefHeartbeat + j, "%2x", &v);
            stream.push_back(static_cast<uint8_t>(v));
        }
        for (size_t j = 0; j < std::strlen(kRefAttitude); j += 2) {
            unsigned v; std::sscanf(kRefAttitude + j, "%2x", &v);
            stream.push_back(static_cast<uint8_t>(v));
        }
        MavMessage m1, m2;
        ParseStatus s1 = rx.decodeNext(stream.data(), stream.size(), m1);
        ParseStatus s2 = rx.decodeNext(nullptr, 0, m2);
        check(s1 == ParseStatus::Complete && m1.msgid == 0
              && s2 == ParseStatus::Complete && m2.msgid == 30,
              "流式解析 (噪声+2帧粘连) 正确");
    }

    // ---- 8) CRC 官方值抽样 (与 docs/mavlink_msg_table.md 对照) ----
    {
        const MsgDef* hb = MavlinkCodec::findDef(0);
        const MsgDef* gps = MavlinkCodec::findDef(24);
        const MsgDef* bat = MavlinkCodec::findDef(147);
        check(hb && hb->crcExtra == 50 && hb->minLen == 9, "HEARTBEAT crc=50 len=9");
        check(gps && gps->crcExtra == 24 && gps->minLen == 30, "GPS_RAW_INT crc=24 min=30");
        check(bat && bat->crcExtra == 154 && bat->minLen == 36, "BATTERY_STATUS crc=154 min=36");
    }

    std::printf("\n%s (%d 项)\n", g_fail ? "存在失败项" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
