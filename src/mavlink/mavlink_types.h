#pragma once
// ============================================================================
// SkyGCS  MAVLink 编解码核心类型定义 (不依赖 Qt, 可独立测试)
// ============================================================================
#include <cstdint>
#include <cstddef>
#include <vector>
#include <string>

namespace skygcs {

// ---------------------------------------------------------------------------
// 字段类型 (MAVLink wire 类型)
// ---------------------------------------------------------------------------
enum class FieldType : uint8_t {
    U8, I8, U16, I16, U32, I32, U64, F32, CHAR
};

// 字段定义: 线上帧内无填充顺序排列 (已按类型长度降序排序, 与部署生态一致)
struct FieldDef {
    FieldType type;
    uint16_t  wireOff;   // 在载荷中的字节偏移
    uint16_t  size;      // 单元素字节数
    uint16_t  count;     // 数组长度, 1 表示标量
};

// 消息定义
struct MsgDef {
    uint32_t        id;
    const char*     name;
    uint8_t         crcExtra;   // 帧校验附加字节 (与官方 c_library_v2 一致)
    uint16_t        minLen;     // 基础载荷长度 (CRC 覆盖范围)
    uint16_t        fullLen;    // 完整载荷长度 (含扩展字段)
    const FieldDef* fields;
    uint16_t        fieldCount;
};

// 一条已解码/待编码的消息
struct MavMessage {
    uint32_t          msgid   = 0;
    uint8_t           sysid   = 1;
    uint8_t           compid  = 1;
    uint8_t           seq     = 0;
    std::vector<uint8_t> payload;
    bool              isV2    = true;

    // 便捷: 载荷访问
    const uint8_t* data() const { return payload.data(); }
    size_t         size() const { return payload.size(); }
};

// 帧解析结果
enum class ParseStatus {
    NeedMore,     // 缓冲区数据不足, 等待更多
    Complete,     // 成功解析出一条完整消息
    BadCrc,       // CRC 校验失败 (丢弃该帧)
    Invalid       // 帧头不合法 (丢弃)
};

// ---------------------------------------------------------------------------
// MAVLink 常量
// ---------------------------------------------------------------------------
namespace mav {
// MAV_TYPE
enum MavType : uint8_t {
    TYPE_GENERIC = 0, TYPE_QUADROTOR = 2, TYPE_HELICOPTER = 3, TYPE_GCS = 6,
};
// MAV_AUTOPILOT
enum MavAutopilot : uint8_t {
    AUTOPILOT_GENERIC = 0, AUTOPILOT_ARDUPILOTMEGA = 3, AUTOPILOT_INVALID = 8,
    AUTOPILOT_PX4 = 12,
};
// MAV_STATE
enum MavState : uint8_t {
    STATE_UNINIT = 0, STATE_BOOT = 1, STATE_CALIBRATING = 2, STATE_STANDBY = 3,
    STATE_ACTIVE = 4, STATE_CRITICAL = 5, STATE_EMERGENCY = 6,
};
// MAV_MODE_FLAG
enum MavModeFlag : uint8_t {
    MODE_FLAG_CUSTOM_MODE_ENABLED = 1, MODE_FLAG_TEST_ENABLED = 2,
    MODE_FLAG_AUTO_ENABLED = 4, MODE_FLAG_GUIDED_ENABLED = 8,
    MODE_FLAG_STABILIZE_ENABLED = 16, MODE_FLAG_HIL_ENABLED = 32,
    MODE_FLAG_MANUAL_INPUT_ENABLED = 64, MODE_FLAG_SAFETY_ARMED = 128,
};
// MAV_CMD (常用)
enum MavCmd : uint16_t {
    CMD_NAV_WAYPOINT = 16, CMD_NAV_LOITER_UNLIM = 17, CMD_NAV_LOITER_TIME = 19,
    CMD_NAV_RETURN_TO_LAUNCH = 20, CMD_NAV_LAND = 21, CMD_NAV_TAKEOFF = 22,
    CMD_NAV_VTOL_TAKEOFF = 84, CMD_DO_SET_MODE = 176, CMD_DO_SET_SERVO = 183,
    CMD_MISSION_START = 300, CMD_PREFLIGHT_CALIBRATION = 241,
    CMD_COMPONENT_ARM_DISARM = 400, CMD_GET_HOME_POSITION = 410,
    CMD_SET_MESSAGE_INTERVAL = 511, CMD_REQUEST_MESSAGE = 512,
};
// MAV_FRAME (航点坐标系)
enum MavFrame : uint8_t {
    FRAME_GLOBAL = 0, FRAME_LOCAL_NED = 1, FRAME_MISSION = 2,
    FRAME_GLOBAL_RELATIVE_ALT = 3, FRAME_LOCAL_ENU = 4, FRAME_GLOBAL_INT = 5,
    FRAME_GLOBAL_RELATIVE_ALT_INT = 6, FRAME_LOCAL_OFFSET_NED = 7,
};
// MAV_MISSION_RESULT
enum MavMissionResult : uint8_t {
    MISSION_ACCEPTED = 0, MISSION_ERROR = 1, MISSION_UNSUPPORTED_FRAME = 2,
    MISSION_UNSUPPORTED = 3, MISSION_NO_SPACE = 4, MISSION_INVALID = 5,
    MISSION_INVALID_PARAM1 = 6, MISSION_INVALID_PARAM2 = 7, MISSION_INVALID_PARAM3 = 8,
    MISSION_INVALID_PARAM4 = 9, MISSION_INVALID_PARAM5_X = 10, MISSION_INVALID_PARAM6_Y = 11,
    MISSION_INVALID_PARAM7 = 12, MISSION_INVALID_SEQUENCE = 13, MISSION_DENIED = 14,
    MISSION_OPERATION_CANCELLED = 15,
};
// MAV_MISSION_TYPE
enum MavMissionType : uint8_t {
    MISSION_TYPE_MISSION = 0, MISSION_TYPE_FENCE = 1, MISSION_TYPE_RALLY = 2,
    MISSION_TYPE_ALL = 255,
};
// MAV_RESULT
enum MavResult : uint8_t {
    RESULT_ACCEPTED = 0, RESULT_TEMPORARILY_REJECTED = 1, RESULT_DENIED = 2,
    RESULT_UNSUPPORTED = 3, RESULT_FAILED = 4, RESULT_IN_PROGRESS = 5,
};
// MAV_PARAM_TYPE (参数值类型)
enum MavParamType : uint8_t {
    PARAM_TYPE_UINT8 = 1, PARAM_TYPE_INT8 = 2, PARAM_TYPE_UINT16 = 3,
    PARAM_TYPE_INT16 = 4, PARAM_TYPE_UINT32 = 5, PARAM_TYPE_INT32 = 6,
    PARAM_TYPE_UINT64 = 7, PARAM_TYPE_INT64 = 8, PARAM_TYPE_REAL32 = 9,
    PARAM_TYPE_REAL64 = 10,
};
// PX4 自定义主模式 (custom_mode 高 8 位)
enum Px4CustomMainMode : uint32_t {
    PX4_MODE_MANUAL = 1, PX4_MODE_ALTCTL = 2, PX4_MODE_POSCTL = 3, PX4_MODE_AUTO = 4,
    PX4_MODE_ACRO = 5, PX4_MODE_OFFBOARD = 6, PX4_MODE_STABILIZED = 7,
    PX4_MODE_RATTITUDE = 8,
};
// PX4 AUTO 子模式 (custom_mode 次高 8 位)
enum Px4AutoSubMode : uint32_t {
    PX4_AUTO_READY = 1, PX4_AUTO_TAKEOFF = 2, PX4_AUTO_LOITER = 3,
    PX4_AUTO_MISSION = 4, PX4_AUTO_RTL = 5, PX4_AUTO_LAND = 6,
};
// STATUSTEXT severity
enum MavSeverity : uint8_t {
    SEVERITY_EMERGENCY = 0, SEVERITY_ALERT = 1, SEVERITY_CRITICAL = 2,
    SEVERITY_ERROR = 3, SEVERITY_WARNING = 4, SEVERITY_NOTICE = 5,
    SEVERITY_INFO = 6, SEVERITY_DEBUG = 7,
};

// PX4 custom_mode 编码: 主模式<<24 | 子模式<<16
inline uint32_t px4CustomModeEncode(uint32_t mainMode, uint32_t subMode = 0) {
    return ((mainMode & 0xFF) << 24) | ((subMode & 0xFF) << 16);
}
inline uint32_t px4CustomModeMain(uint32_t customMode) { return (customMode >> 24) & 0xFF; }
inline uint32_t px4CustomModeSub(uint32_t customMode)   { return (customMode >> 16) & 0xFF; }
} // namespace mav

} // namespace skygcs
