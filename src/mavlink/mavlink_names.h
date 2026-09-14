#pragma once
// ============================================================================
// SkyGCS  MAVLink 名称辅助 (UI 层用, 依赖 Qt)
// 与无 Qt 的编解码核心 (mavlink_types.h) 分离, 保持协议层可独立测试
// ============================================================================
#include <QString>

#include "mavlink_types.h"

namespace skygcs {
namespace mav {

inline QString commandName(uint16_t cmd) {
    switch (cmd) {
    case CMD_NAV_WAYPOINT: return QStringLiteral("NAV_WAYPOINT");
    case CMD_NAV_LOITER_UNLIM: return QStringLiteral("NAV_LOITER_UNLIM");
    case CMD_NAV_LOITER_TIME: return QStringLiteral("NAV_LOITER_TIME");
    case CMD_NAV_RETURN_TO_LAUNCH: return QStringLiteral("NAV_RETURN_TO_LAUNCH");
    case CMD_NAV_LAND: return QStringLiteral("NAV_LAND");
    case CMD_NAV_TAKEOFF: return QStringLiteral("NAV_TAKEOFF");
    case CMD_DO_SET_MODE: return QStringLiteral("DO_SET_MODE");
    case CMD_DO_SET_SERVO: return QStringLiteral("DO_SET_SERVO");
    case CMD_PREFLIGHT_CALIBRATION: return QStringLiteral("PREFLIGHT_CALIBRATION");
    case CMD_COMPONENT_ARM_DISARM: return QStringLiteral("COMPONENT_ARM_DISARM");
    case CMD_GET_HOME_POSITION: return QStringLiteral("GET_HOME_POSITION");
    case CMD_SET_MESSAGE_INTERVAL: return QStringLiteral("SET_MESSAGE_INTERVAL");
    case CMD_REQUEST_MESSAGE: return QStringLiteral("REQUEST_MESSAGE");
    default: return QStringLiteral("CMD_%1").arg(cmd);
    }
}

inline QString commandResultName(uint8_t r) {
    switch (r) {
    case RESULT_ACCEPTED: return QStringLiteral("ACCEPTED");
    case RESULT_TEMPORARILY_REJECTED: return QStringLiteral("TEMPORARILY_REJECTED");
    case RESULT_DENIED: return QStringLiteral("DENIED");
    case RESULT_UNSUPPORTED: return QStringLiteral("UNSUPPORTED");
    case RESULT_FAILED: return QStringLiteral("FAILED");
    case RESULT_IN_PROGRESS: return QStringLiteral("IN_PROGRESS");
    default: return QStringLiteral("RESULT_%1").arg(r);
    }
}

} // namespace mav
} // namespace skygcs
