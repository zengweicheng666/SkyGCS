#pragma once
// ============================================================================
// SkyGCS  MAVLink 常用消息结构体
// 字段顺序 == 线上帧字段序 (类型长度降序, 与官方 c_library_v2 一致)
// 因此 C 结构体布局 == wire 布局 (无填充), pack/unpack 可直接按表拷贝
// ============================================================================
#include <cstdint>
#include <cstring>

#include "mavlink_codec.h"

// 结构体布局 == wire 布局 (无填充): 保证 pack/unpack 按偏移拷贝正确,
// 且 sizeof 与官方 MIN_LEN 一致 (MSVC 默认与 GCC 需显式 pack)
#pragma pack(push, 1)
namespace skygcs {

// HEARTBEAT (id=0)
struct HeartbeatMsg {
    uint32_t custom_mode;
    uint8_t  type;
    uint8_t  autopilot;
    uint8_t  base_mode;
    uint8_t  system_status;
    uint8_t  mavlink_version;
};
static_assert(sizeof(HeartbeatMsg) == 9, "layout");

// SYS_STATUS (id=1, 基础 31B)
struct SysStatusMsg {
    uint32_t onboard_control_sensors_present;
    uint32_t onboard_control_sensors_enabled;
    uint32_t onboard_control_sensors_health;
    uint16_t load;               // 0-1000
    uint16_t voltage_battery;    // mV
    int16_t  current_battery;    // cA
    uint16_t drop_rate_comm;     // c%
    uint16_t errors_comm;
    uint16_t errors_count1;
    uint16_t errors_count2;
    uint16_t errors_count3;
    uint16_t errors_count4;
    int8_t   battery_remaining;  // %
};

// SYSTEM_TIME (id=2)
struct SystemTimeMsg {
    uint64_t time_unix_usec;
    uint32_t time_boot_ms;
};

// PING (id=4)
struct PingMsg {
    uint64_t time_usec;
    uint32_t seq;
    uint8_t  target_system;
    uint8_t  target_component;
};

// SET_MODE (id=11)
struct SetModeMsg {
    uint32_t custom_mode;
    uint8_t  target_system;
    uint8_t  base_mode;
};

// PARAM_REQUEST_LIST (id=21)
struct ParamRequestListMsg {
    uint8_t target_system;
    uint8_t target_component;
};

// PARAM_VALUE (id=22)
struct ParamValueMsg {
    float    param_value;
    uint16_t param_count;
    uint16_t param_index;
    char     param_id[16];
    uint8_t  param_type;
};

// GPS_RAW_INT (id=24)
struct GpsRawIntMsg {
    uint64_t time_usec;
    int32_t  lat;               // degE7
    int32_t  lon;
    int32_t  alt;               // mm (MSL)
    uint16_t eph;               // cm
    uint16_t epv;
    uint16_t vel;               // cm/s
    uint16_t cog;               // cdeg
    uint8_t  fix_type;
    uint8_t  satellites_visible;
};

// ATTITUDE (id=30)
struct AttitudeMsg {
    uint32_t time_boot_ms;
    float    roll;              // rad
    float    pitch;
    float    yaw;
    float    rollspeed;         // rad/s
    float    pitchspeed;
    float    yawspeed;
};

// LOCAL_POSITION_NED (id=32)
struct LocalPositionNedMsg {
    uint32_t time_boot_ms;
    float    x;
    float    y;
    float    z;
    float    vx;
    float    vy;
    float    vz;
};

// GLOBAL_POSITION_INT (id=33)
struct GlobalPositionIntMsg {
    uint32_t time_boot_ms;
    int32_t  lat;               // degE7
    int32_t  lon;
    int32_t  alt;               // mm (MSL)
    int32_t  relative_alt;      // mm (相对起飞点)
    int16_t  vx;                // cm/s (NED)
    int16_t  vy;
    int16_t  vz;
    uint16_t hdg;               // cdeg
};

// RC_CHANNELS_RAW (id=35)
struct RcChannelsRawMsg {
    uint32_t time_boot_ms;
    uint16_t chan1_raw, chan2_raw, chan3_raw, chan4_raw;
    uint16_t chan5_raw, chan6_raw, chan7_raw, chan8_raw;
    uint8_t  port;
    uint8_t  rssi;
};

// VFR_HUD (id=74)
struct VfrHudMsg {
    float    airspeed;          // m/s
    float    groundspeed;
    float    alt;               // m
    float    climb;             // m/s
    int16_t  heading;           // deg
    uint16_t throttle;          // 0-100%
};

// COMMAND_INT (id=75)
struct CommandIntMsg {
    float    param1, param2, param3, param4;
    int32_t  x;
    int32_t  y;
    float    z;
    uint16_t command;
    uint8_t  target_system;
    uint8_t  target_component;
    uint8_t  frame;
    uint8_t  current;
    uint8_t  autocontinue;
};

// COMMAND_LONG (id=76)
struct CommandLongMsg {
    float    param1, param2, param3, param4;
    float    param5, param6, param7;
    uint16_t command;
    uint8_t  target_system;
    uint8_t  target_component;
    uint8_t  confirmation;
};

// COMMAND_ACK (id=77) —— 含扩展字段 (progress/result_param2), 手动解包
struct CommandAckMsg {
    uint16_t command;
    uint8_t  result;
    uint8_t  progress;
    int32_t  result_param2;
    uint8_t  target_system;
    uint8_t  target_component;
};

// HIL_ACTUATOR_CONTROLS (id=93)
struct HilActuatorControlsMsg {
    uint64_t time_usec;
    uint64_t flags;
    float    controls[16];
    uint8_t  mode;
};

// HIL_SENSOR (id=107)
struct HilSensorMsg {
    uint64_t time_usec;
    float    xacc, yacc, zacc;
    float    xgyro, ygyro, zgyro;
    float    xmag, ymag, zmag;
    float    abs_pressure, diff_pressure, pressure_alt;
    float    temperature;
    uint32_t fields_updated;
};

// RADIO_STATUS (id=109)
struct RadioStatusMsg {
    uint16_t rxerrors;
    uint16_t fixed;
    uint8_t  rssi;
    uint8_t  remrssi;
    uint8_t  txbuf;
    uint8_t  noise;
    uint8_t  remnoise;
};

// HIL_GPS (id=113)
struct HilGpsMsg {
    uint64_t time_usec;
    int32_t  lat;
    int32_t  lon;
    int32_t  alt;
    uint16_t eph;
    uint16_t epv;
    uint16_t vel;
    int16_t  vn;
    int16_t  ve;
    int16_t  vd;
    uint16_t cog;
    uint8_t  fix_type;
    uint8_t  satellites_visible;
};

// HIL_STATE_QUATERNION (id=115)
struct HilStateQuaternionMsg {
    uint64_t time_usec;
    float    q[4];
    float    rollspeed, pitchspeed, yawspeed;
    int32_t  lat, lon, alt;
    int16_t  vx, vy, vz;
    uint16_t ind_airspeed, true_airspeed;
    int16_t  xacc, yacc, zacc;
};

// BATTERY_STATUS (id=147)
struct BatteryStatusMsg {
    int32_t  current_consumed;   // mAh
    int32_t  energy_consumed;    // hJ
    int16_t  temperature;        // cdegC
    uint16_t voltages[10];       // mV
    int16_t  current_battery;    // cA
    uint8_t  id;
    uint8_t  battery_function;
    uint8_t  type;
    int8_t   battery_remaining;  // %
};

// HOME_POSITION (id=242)
struct HomePositionMsg {
    int32_t latitude;            // degE7
    int32_t longitude;
    int32_t altitude;            // mm
    float   x, y, z;
    float   q[4];
    float   approach_x, approach_y, approach_z;
};

// STATUSTEXT (id=253)
struct StatustextMsg {
    uint8_t severity;
    char    text[50];
};

// ---------------------------------------------------------------------------
// 便捷打包/解包
// ---------------------------------------------------------------------------
inline bool packHeartbeat(uint8_t type, uint8_t autopilot, uint8_t baseMode,
                          uint32_t customMode, uint8_t sysStatus, MavMessage& out)
{
    HeartbeatMsg m{};
    m.type = type; m.autopilot = autopilot; m.base_mode = baseMode;
    m.custom_mode = customMode; m.system_status = sysStatus;
    m.mavlink_version = 3;
    return MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_HEARTBEAT), &m, out);
}

inline bool unpackHeartbeat(const MavMessage& msg, HeartbeatMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_HEARTBEAT);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackAttitude(const MavMessage& msg, AttitudeMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_ATTITUDE);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackGlobalPositionInt(const MavMessage& msg, GlobalPositionIntMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_GLOBAL_POSITION_INT);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackGpsRawInt(const MavMessage& msg, GpsRawIntMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_GPS_RAW_INT);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackVfrHud(const MavMessage& msg, VfrHudMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_VFR_HUD);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackBatteryStatus(const MavMessage& msg, BatteryStatusMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_BATTERY_STATUS);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackSysStatus(const MavMessage& msg, SysStatusMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_SYS_STATUS);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackStatustext(const MavMessage& msg, StatustextMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_STATUSTEXT);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackHomePosition(const MavMessage& msg, HomePositionMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_HOME_POSITION);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackRadioStatus(const MavMessage& msg, RadioStatusMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_RADIO_STATUS);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

inline bool unpackLocalPositionNed(const MavMessage& msg, LocalPositionNedMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_LOCAL_POSITION_NED);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

// COMMAND_ACK 完整解包 (含扩展字段 progress/result_param2)
inline bool unpackCommandAck(const MavMessage& msg, CommandAckMsg& out)
{
    if (msg.msgid != MAV_MSG_ID_COMMAND_ACK || msg.size() < 3)
        return false;
    std::memset(&out, 0, sizeof(out));
    const uint8_t* p = msg.data();
    out.command = static_cast<uint16_t>(p[0] | (p[1] << 8));
    out.result = p[2];
    if (msg.size() >= 4) out.progress = p[3];
    if (msg.size() >= 8) out.result_param2 = static_cast<int32_t>(
        static_cast<uint32_t>(p[4]) | (static_cast<uint32_t>(p[5]) << 8)
        | (static_cast<uint32_t>(p[6]) << 16) | (static_cast<uint32_t>(p[7]) << 24));
    if (msg.size() >= 9) out.target_system = p[8];
    if (msg.size() >= 10) out.target_component = p[9];
    return true;
}

inline bool unpackPing(const MavMessage& msg, PingMsg& out)
{
    const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_PING);
    return def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &out);
}

} // namespace skygcs
#pragma pack(pop)
