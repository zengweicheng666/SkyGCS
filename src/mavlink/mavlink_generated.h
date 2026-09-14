// ============================================================================
// 本文件由 tools/gen_mavlink.py 自动生成 —— 请勿手工修改
// 字段序/CRC/LEN 提取自官方 c_library_v2 (github.com/mavlink/c_library_v2)
// 已按 mavlink C 库 crc_accumulate 算法交叉验证
// 生成时间: 2026-09-14 18:48:57
// ============================================================================
#pragma once
#include <cstdint>
#include <cstddef>
#include "mavlink_types.h"

namespace skygcs {

// 消息 ID
enum MavMsgId : uint32_t {
    MAV_MSG_ID_HEARTBEAT = 0,
    MAV_MSG_ID_SYS_STATUS = 1,
    MAV_MSG_ID_SYSTEM_TIME = 2,
    MAV_MSG_ID_PING = 4,
    MAV_MSG_ID_SET_MODE = 11,
    MAV_MSG_ID_PARAM_REQUEST_LIST = 21,
    MAV_MSG_ID_PARAM_VALUE = 22,
    MAV_MSG_ID_GPS_RAW_INT = 24,
    MAV_MSG_ID_ATTITUDE = 30,
    MAV_MSG_ID_LOCAL_POSITION_NED = 32,
    MAV_MSG_ID_GLOBAL_POSITION_INT = 33,
    MAV_MSG_ID_RC_CHANNELS_RAW = 35,
    MAV_MSG_ID_MISSION_CURRENT = 42,
    MAV_MSG_ID_MISSION_COUNT = 44,
    MAV_MSG_ID_MISSION_ITEM_REACHED = 46,
    MAV_MSG_ID_MISSION_ACK = 47,
    MAV_MSG_ID_MISSION_REQUEST_INT = 51,
    MAV_MSG_ID_MISSION_ITEM_INT = 73,
    MAV_MSG_ID_VFR_HUD = 74,
    MAV_MSG_ID_COMMAND_INT = 75,
    MAV_MSG_ID_COMMAND_LONG = 76,
    MAV_MSG_ID_COMMAND_ACK = 77,
    MAV_MSG_ID_HIL_ACTUATOR_CONTROLS = 93,
    MAV_MSG_ID_HIL_SENSOR = 107,
    MAV_MSG_ID_RADIO_STATUS = 109,
    MAV_MSG_ID_HIL_GPS = 113,
    MAV_MSG_ID_HIL_STATE_QUATERNION = 115,
    MAV_MSG_ID_BATTERY_STATUS = 147,
    MAV_MSG_ID_HOME_POSITION = 242,
    MAV_MSG_ID_STATUSTEXT = 253,
};

// HEARTBEAT: id=0, crc_extra=50, payload=9B
static constexpr FieldDef kFields_HEARTBEAT[] = {
    { FieldType::U32, 0, 4 },
    { FieldType::U8, 4, 1 },
    { FieldType::U8, 5, 1 },
    { FieldType::U8, 6, 1 },
    { FieldType::U8, 7, 1 },
    { FieldType::U8, 8, 1 },
};

// SYS_STATUS: id=1, crc_extra=124, payload=43B
static constexpr FieldDef kFields_SYS_STATUS[] = {
    { FieldType::U32, 0, 4 },
    { FieldType::U32, 4, 4 },
    { FieldType::U32, 8, 4 },
    { FieldType::U16, 12, 2 },
    { FieldType::U16, 14, 2 },
    { FieldType::I16, 16, 2 },
    { FieldType::U16, 18, 2 },
    { FieldType::U16, 20, 2 },
    { FieldType::U16, 22, 2 },
    { FieldType::U16, 24, 2 },
    { FieldType::U16, 26, 2 },
    { FieldType::U16, 28, 2 },
    { FieldType::I8, 30, 1 },
};

// SYSTEM_TIME: id=2, crc_extra=137, payload=12B
static constexpr FieldDef kFields_SYSTEM_TIME[] = {
    { FieldType::U64, 0, 8 },
    { FieldType::U32, 8, 4 },
};

// PING: id=4, crc_extra=237, payload=14B
static constexpr FieldDef kFields_PING[] = {
    { FieldType::U64, 0, 8 },
    { FieldType::U32, 8, 4 },
    { FieldType::U8, 12, 1 },
    { FieldType::U8, 13, 1 },
};

// SET_MODE: id=11, crc_extra=89, payload=6B
static constexpr FieldDef kFields_SET_MODE[] = {
    { FieldType::U32, 0, 4 },
    { FieldType::U8, 4, 1 },
    { FieldType::U8, 5, 1 },
};

// PARAM_REQUEST_LIST: id=21, crc_extra=159, payload=2B
static constexpr FieldDef kFields_PARAM_REQUEST_LIST[] = {
    { FieldType::U8, 0, 1 },
    { FieldType::U8, 1, 1 },
};

// PARAM_VALUE: id=22, crc_extra=220, payload=25B
static constexpr FieldDef kFields_PARAM_VALUE[] = {
    { FieldType::F32, 0, 4 },
    { FieldType::U16, 4, 2 },
    { FieldType::U16, 6, 2 },
    { FieldType::CHAR, 8, 1, 16 },
    { FieldType::U8, 24, 1 },
};

// GPS_RAW_INT: id=24, crc_extra=24, payload=52B
static constexpr FieldDef kFields_GPS_RAW_INT[] = {
    { FieldType::U64, 0, 8 },
    { FieldType::I32, 8, 4 },
    { FieldType::I32, 12, 4 },
    { FieldType::I32, 16, 4 },
    { FieldType::U16, 20, 2 },
    { FieldType::U16, 22, 2 },
    { FieldType::U16, 24, 2 },
    { FieldType::U16, 26, 2 },
    { FieldType::U8, 28, 1 },
    { FieldType::U8, 29, 1 },
};

// ATTITUDE: id=30, crc_extra=39, payload=28B
static constexpr FieldDef kFields_ATTITUDE[] = {
    { FieldType::U32, 0, 4 },
    { FieldType::F32, 4, 4 },
    { FieldType::F32, 8, 4 },
    { FieldType::F32, 12, 4 },
    { FieldType::F32, 16, 4 },
    { FieldType::F32, 20, 4 },
    { FieldType::F32, 24, 4 },
};

// LOCAL_POSITION_NED: id=32, crc_extra=185, payload=28B
static constexpr FieldDef kFields_LOCAL_POSITION_NED[] = {
    { FieldType::U32, 0, 4 },
    { FieldType::F32, 4, 4 },
    { FieldType::F32, 8, 4 },
    { FieldType::F32, 12, 4 },
    { FieldType::F32, 16, 4 },
    { FieldType::F32, 20, 4 },
    { FieldType::F32, 24, 4 },
};

// GLOBAL_POSITION_INT: id=33, crc_extra=104, payload=28B
static constexpr FieldDef kFields_GLOBAL_POSITION_INT[] = {
    { FieldType::U32, 0, 4 },
    { FieldType::I32, 4, 4 },
    { FieldType::I32, 8, 4 },
    { FieldType::I32, 12, 4 },
    { FieldType::I32, 16, 4 },
    { FieldType::I16, 20, 2 },
    { FieldType::I16, 22, 2 },
    { FieldType::I16, 24, 2 },
    { FieldType::U16, 26, 2 },
};

// RC_CHANNELS_RAW: id=35, crc_extra=244, payload=22B
static constexpr FieldDef kFields_RC_CHANNELS_RAW[] = {
    { FieldType::U32, 0, 4 },
    { FieldType::U16, 4, 2 },
    { FieldType::U16, 6, 2 },
    { FieldType::U16, 8, 2 },
    { FieldType::U16, 10, 2 },
    { FieldType::U16, 12, 2 },
    { FieldType::U16, 14, 2 },
    { FieldType::U16, 16, 2 },
    { FieldType::U16, 18, 2 },
    { FieldType::U8, 20, 1 },
    { FieldType::U8, 21, 1 },
};

// MISSION_CURRENT: id=42, crc_extra=28, payload=18B
static constexpr FieldDef kFields_MISSION_CURRENT[] = {
    { FieldType::U16, 0, 2 },
};

// MISSION_COUNT: id=44, crc_extra=221, payload=9B
static constexpr FieldDef kFields_MISSION_COUNT[] = {
    { FieldType::U16, 0, 2 },
    { FieldType::U8, 2, 1 },
    { FieldType::U8, 3, 1 },
};

// MISSION_ITEM_REACHED: id=46, crc_extra=11, payload=2B
static constexpr FieldDef kFields_MISSION_ITEM_REACHED[] = {
    { FieldType::U16, 0, 2 },
};

// MISSION_ACK: id=47, crc_extra=153, payload=8B
static constexpr FieldDef kFields_MISSION_ACK[] = {
    { FieldType::U8, 0, 1 },
    { FieldType::U8, 1, 1 },
    { FieldType::U8, 2, 1 },
};

// MISSION_REQUEST_INT: id=51, crc_extra=196, payload=5B
static constexpr FieldDef kFields_MISSION_REQUEST_INT[] = {
    { FieldType::U16, 0, 2 },
    { FieldType::U8, 2, 1 },
    { FieldType::U8, 3, 1 },
};

// MISSION_ITEM_INT: id=73, crc_extra=38, payload=38B
static constexpr FieldDef kFields_MISSION_ITEM_INT[] = {
    { FieldType::F32, 0, 4 },
    { FieldType::F32, 4, 4 },
    { FieldType::F32, 8, 4 },
    { FieldType::F32, 12, 4 },
    { FieldType::I32, 16, 4 },
    { FieldType::I32, 20, 4 },
    { FieldType::F32, 24, 4 },
    { FieldType::U16, 28, 2 },
    { FieldType::U16, 30, 2 },
    { FieldType::U8, 32, 1 },
    { FieldType::U8, 33, 1 },
    { FieldType::U8, 34, 1 },
    { FieldType::U8, 35, 1 },
    { FieldType::U8, 36, 1 },
};

// VFR_HUD: id=74, crc_extra=20, payload=20B
static constexpr FieldDef kFields_VFR_HUD[] = {
    { FieldType::F32, 0, 4 },
    { FieldType::F32, 4, 4 },
    { FieldType::F32, 8, 4 },
    { FieldType::F32, 12, 4 },
    { FieldType::I16, 16, 2 },
    { FieldType::U16, 18, 2 },
};

// COMMAND_INT: id=75, crc_extra=158, payload=35B
static constexpr FieldDef kFields_COMMAND_INT[] = {
    { FieldType::F32, 0, 4 },
    { FieldType::F32, 4, 4 },
    { FieldType::F32, 8, 4 },
    { FieldType::F32, 12, 4 },
    { FieldType::I32, 16, 4 },
    { FieldType::I32, 20, 4 },
    { FieldType::F32, 24, 4 },
    { FieldType::U16, 28, 2 },
    { FieldType::U8, 30, 1 },
    { FieldType::U8, 31, 1 },
    { FieldType::U8, 32, 1 },
    { FieldType::U8, 33, 1 },
    { FieldType::U8, 34, 1 },
};

// COMMAND_LONG: id=76, crc_extra=152, payload=33B
static constexpr FieldDef kFields_COMMAND_LONG[] = {
    { FieldType::F32, 0, 4 },
    { FieldType::F32, 4, 4 },
    { FieldType::F32, 8, 4 },
    { FieldType::F32, 12, 4 },
    { FieldType::F32, 16, 4 },
    { FieldType::F32, 20, 4 },
    { FieldType::F32, 24, 4 },
    { FieldType::U16, 28, 2 },
    { FieldType::U8, 30, 1 },
    { FieldType::U8, 31, 1 },
    { FieldType::U8, 32, 1 },
};

// COMMAND_ACK: id=77, crc_extra=143, payload=10B
static constexpr FieldDef kFields_COMMAND_ACK[] = {
    { FieldType::U16, 0, 2 },
    { FieldType::U8, 2, 1 },
};

// HIL_ACTUATOR_CONTROLS: id=93, crc_extra=47, payload=81B
static constexpr FieldDef kFields_HIL_ACTUATOR_CONTROLS[] = {
    { FieldType::U64, 0, 8 },
    { FieldType::U64, 8, 8 },
    { FieldType::F32, 16, 4, 16 },
    { FieldType::U8, 80, 1 },
};

// HIL_SENSOR: id=107, crc_extra=108, payload=65B
static constexpr FieldDef kFields_HIL_SENSOR[] = {
    { FieldType::U64, 0, 8 },
    { FieldType::F32, 8, 4 },
    { FieldType::F32, 12, 4 },
    { FieldType::F32, 16, 4 },
    { FieldType::F32, 20, 4 },
    { FieldType::F32, 24, 4 },
    { FieldType::F32, 28, 4 },
    { FieldType::F32, 32, 4 },
    { FieldType::F32, 36, 4 },
    { FieldType::F32, 40, 4 },
    { FieldType::F32, 44, 4 },
    { FieldType::F32, 48, 4 },
    { FieldType::F32, 52, 4 },
    { FieldType::F32, 56, 4 },
    { FieldType::U32, 60, 4 },
};

// RADIO_STATUS: id=109, crc_extra=185, payload=9B
static constexpr FieldDef kFields_RADIO_STATUS[] = {
    { FieldType::U16, 0, 2 },
    { FieldType::U16, 2, 2 },
    { FieldType::U8, 4, 1 },
    { FieldType::U8, 5, 1 },
    { FieldType::U8, 6, 1 },
    { FieldType::U8, 7, 1 },
    { FieldType::U8, 8, 1 },
};

// HIL_GPS: id=113, crc_extra=124, payload=39B
static constexpr FieldDef kFields_HIL_GPS[] = {
    { FieldType::U64, 0, 8 },
    { FieldType::I32, 8, 4 },
    { FieldType::I32, 12, 4 },
    { FieldType::I32, 16, 4 },
    { FieldType::U16, 20, 2 },
    { FieldType::U16, 22, 2 },
    { FieldType::U16, 24, 2 },
    { FieldType::I16, 26, 2 },
    { FieldType::I16, 28, 2 },
    { FieldType::I16, 30, 2 },
    { FieldType::U16, 32, 2 },
    { FieldType::U8, 34, 1 },
    { FieldType::U8, 35, 1 },
};

// HIL_STATE_QUATERNION: id=115, crc_extra=4, payload=64B
static constexpr FieldDef kFields_HIL_STATE_QUATERNION[] = {
    { FieldType::U64, 0, 8 },
    { FieldType::F32, 8, 4, 4 },
    { FieldType::F32, 24, 4 },
    { FieldType::F32, 28, 4 },
    { FieldType::F32, 32, 4 },
    { FieldType::I32, 36, 4 },
    { FieldType::I32, 40, 4 },
    { FieldType::I32, 44, 4 },
    { FieldType::I16, 48, 2 },
    { FieldType::I16, 50, 2 },
    { FieldType::I16, 52, 2 },
    { FieldType::U16, 54, 2 },
    { FieldType::U16, 56, 2 },
    { FieldType::I16, 58, 2 },
    { FieldType::I16, 60, 2 },
    { FieldType::I16, 62, 2 },
};

// BATTERY_STATUS: id=147, crc_extra=154, payload=54B
static constexpr FieldDef kFields_BATTERY_STATUS[] = {
    { FieldType::I32, 0, 4 },
    { FieldType::I32, 4, 4 },
    { FieldType::I16, 8, 2 },
    { FieldType::U16, 10, 2, 10 },
    { FieldType::I16, 30, 2 },
    { FieldType::U8, 32, 1 },
    { FieldType::U8, 33, 1 },
    { FieldType::U8, 34, 1 },
    { FieldType::I8, 35, 1 },
};

// HOME_POSITION: id=242, crc_extra=104, payload=60B
static constexpr FieldDef kFields_HOME_POSITION[] = {
    { FieldType::I32, 0, 4 },
    { FieldType::I32, 4, 4 },
    { FieldType::I32, 8, 4 },
    { FieldType::F32, 12, 4 },
    { FieldType::F32, 16, 4 },
    { FieldType::F32, 20, 4 },
    { FieldType::F32, 24, 4, 4 },
    { FieldType::F32, 40, 4 },
    { FieldType::F32, 44, 4 },
    { FieldType::F32, 48, 4 },
};

// STATUSTEXT: id=253, crc_extra=83, payload=54B
static constexpr FieldDef kFields_STATUSTEXT[] = {
    { FieldType::U8, 0, 1 },
    { FieldType::CHAR, 1, 1, 50 },
};

// 消息定义表
static constexpr MsgDef kMsgDefs[] = {
    { 0, "HEARTBEAT", 50, 9, 9, kFields_HEARTBEAT, 6 },
    { 1, "SYS_STATUS", 124, 31, 43, kFields_SYS_STATUS, 13 },
    { 2, "SYSTEM_TIME", 137, 12, 12, kFields_SYSTEM_TIME, 2 },
    { 4, "PING", 237, 14, 14, kFields_PING, 4 },
    { 11, "SET_MODE", 89, 6, 6, kFields_SET_MODE, 3 },
    { 21, "PARAM_REQUEST_LIST", 159, 2, 2, kFields_PARAM_REQUEST_LIST, 2 },
    { 22, "PARAM_VALUE", 220, 25, 25, kFields_PARAM_VALUE, 5 },
    { 24, "GPS_RAW_INT", 24, 30, 52, kFields_GPS_RAW_INT, 10 },
    { 30, "ATTITUDE", 39, 28, 28, kFields_ATTITUDE, 7 },
    { 32, "LOCAL_POSITION_NED", 185, 28, 28, kFields_LOCAL_POSITION_NED, 7 },
    { 33, "GLOBAL_POSITION_INT", 104, 28, 28, kFields_GLOBAL_POSITION_INT, 9 },
    { 35, "RC_CHANNELS_RAW", 244, 22, 22, kFields_RC_CHANNELS_RAW, 11 },
    { 42, "MISSION_CURRENT", 28, 2, 18, kFields_MISSION_CURRENT, 1 },
    { 44, "MISSION_COUNT", 221, 4, 9, kFields_MISSION_COUNT, 3 },
    { 46, "MISSION_ITEM_REACHED", 11, 2, 2, kFields_MISSION_ITEM_REACHED, 1 },
    { 47, "MISSION_ACK", 153, 3, 8, kFields_MISSION_ACK, 3 },
    { 51, "MISSION_REQUEST_INT", 196, 4, 5, kFields_MISSION_REQUEST_INT, 3 },
    { 73, "MISSION_ITEM_INT", 38, 37, 38, kFields_MISSION_ITEM_INT, 14 },
    { 74, "VFR_HUD", 20, 20, 20, kFields_VFR_HUD, 6 },
    { 75, "COMMAND_INT", 158, 35, 35, kFields_COMMAND_INT, 13 },
    { 76, "COMMAND_LONG", 152, 33, 33, kFields_COMMAND_LONG, 11 },
    { 77, "COMMAND_ACK", 143, 3, 10, kFields_COMMAND_ACK, 2 },
    { 93, "HIL_ACTUATOR_CONTROLS", 47, 81, 81, kFields_HIL_ACTUATOR_CONTROLS, 4 },
    { 107, "HIL_SENSOR", 108, 64, 65, kFields_HIL_SENSOR, 15 },
    { 109, "RADIO_STATUS", 185, 9, 9, kFields_RADIO_STATUS, 7 },
    { 113, "HIL_GPS", 124, 36, 39, kFields_HIL_GPS, 13 },
    { 115, "HIL_STATE_QUATERNION", 4, 64, 64, kFields_HIL_STATE_QUATERNION, 16 },
    { 147, "BATTERY_STATUS", 154, 36, 54, kFields_BATTERY_STATUS, 9 },
    { 242, "HOME_POSITION", 104, 52, 60, kFields_HOME_POSITION, 10 },
    { 253, "STATUSTEXT", 83, 51, 54, kFields_STATUSTEXT, 2 },
};
static constexpr size_t kMsgDefCount = 30;

} // namespace skygcs
