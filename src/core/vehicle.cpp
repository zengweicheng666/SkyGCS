#include "vehicle.h"

#include <QElapsedTimer>
#include <cmath>

#include "../mavlink/mavlink_types.h"

namespace skygcs {

VehicleState::VehicleState(QObject* parent)
    : QObject(parent)
{
}

int VehicleState::heartbeatAgeMs() const
{
    return static_cast<int>(QDateTime::currentMSecsSinceEpoch() - lastHeartbeatMs_);
}

QString VehicleState::typeName() const
{
    switch (type_) {
    case mav::TYPE_QUADROTOR: return QObject::tr("四旋翼");
    case mav::TYPE_HELICOPTER: return QObject::tr("直升机");
    case mav::TYPE_GCS: return QObject::tr("地面站");
    default: return QObject::tr("通用飞行器");
    }
}

QString VehicleState::autopilotName() const
{
    switch (autopilot_) {
    case mav::AUTOPILOT_PX4: return "PX4";
    case mav::AUTOPILOT_ARDUPILOTMEGA: return "ArduPilot";
    case mav::AUTOPILOT_INVALID: return "N/A";
    default: return QObject::tr("通用");
    }
}

QString VehicleState::modeName() const
{
    if (autopilot_ != mav::AUTOPILOT_PX4) {
        // 通用: 依据 base_mode 标志给出可读描述
        QStringList flags;
        if (baseMode_ & mav::MODE_FLAG_SAFETY_ARMED) flags << "ARMED";
        if (baseMode_ & mav::MODE_FLAG_AUTO_ENABLED) flags << "AUTO";
        if (baseMode_ & mav::MODE_FLAG_GUIDED_ENABLED) flags << "GUIDED";
        if (baseMode_ & mav::MODE_FLAG_STABILIZE_ENABLED) flags << "STAB";
        if (baseMode_ & mav::MODE_FLAG_MANUAL_INPUT_ENABLED) flags << "MANUAL";
        return flags.isEmpty() ? tr("UNKNOWN") : flags.join("|");
    }
    const uint32_t main = mav::px4CustomModeMain(customMode_);
    const uint32_t sub = mav::px4CustomModeSub(customMode_);
    switch (main) {
    case mav::PX4_MODE_MANUAL: return "手动 Manual";
    case mav::PX4_MODE_ALTCTL: return "定高 AltCtl";
    case mav::PX4_MODE_POSCTL: return "定点 PosCtl";
    case mav::PX4_MODE_AUTO:
        switch (sub) {
        case mav::PX4_AUTO_READY: return "自动-就绪 Auto Ready";
        case mav::PX4_AUTO_TAKEOFF: return "自动-起飞 Auto Takeoff";
        case mav::PX4_AUTO_LOITER: return "自动-悬停 Auto Loiter";
        case mav::PX4_AUTO_MISSION: return "自动-任务 Auto Mission";
        case mav::PX4_AUTO_RTL: return "自动-返航 Auto RTL";
        case mav::PX4_AUTO_LAND: return "自动-降落 Auto Land";
        default: return "自动 Auto";
        }
    case mav::PX4_MODE_ACRO: return "特技 Acro";
    case mav::PX4_MODE_OFFBOARD: return "机载 Offboard";
    case mav::PX4_MODE_STABILIZED: return "自稳 Stabilized";
    case mav::PX4_MODE_RATTITUDE: return "角速度 Rattitude";
    default: return tr("未知模式(0x%1)").arg(customMode_, 8, 16, QChar('0'));
    }
}

QString VehicleState::systemStatusName() const
{
    switch (systemStatus_) {
    case mav::STATE_STANDBY: return tr("待机");
    case mav::STATE_ACTIVE: return tr("活动");
    case mav::STATE_CRITICAL: return tr("严重");
    case mav::STATE_EMERGENCY: return tr("紧急");
    case mav::STATE_CALIBRATING: return tr("校准中");
    default: return tr("未初始化");
    }
}

QString VehicleState::gpsFixName() const
{
    switch (gpsFix_) {
    case 1: return tr("无定位");
    case 2: return tr("2D");
    case 3: return tr("3D");
    case 4: return tr("3D+RTK浮点");
    case 5: return tr("3D+RTK固定");
    default: return tr("无GPS");
    }
}

void VehicleState::updateHeartbeat(uint8_t sysid, uint8_t compid, uint8_t type,
                                   uint8_t autopilot, uint8_t baseMode,
                                   uint32_t customMode, uint8_t systemStatus)
{
    const bool wasOnline = online_;
    online_ = true;
    sysid_ = sysid;
    compid_ = compid;
    type_ = type;
    autopilot_ = autopilot;
    baseMode_ = baseMode;
    customMode_ = customMode;
    systemStatus_ = systemStatus;
    armed_ = (baseMode & mav::MODE_FLAG_SAFETY_ARMED) != 0;
    lastHeartbeatMs_ = QDateTime::currentMSecsSinceEpoch();
    if (!wasOnline)
        emit connectionChanged(true);
    emit stateChanged();
}

void VehicleState::updateAttitude(double roll, double pitch, double yaw,
                                  double rollRate, double pitchRate, double yawRate)
{
    rollDeg_ = roll * 180.0 / M_PI;
    pitchDeg_ = pitch * 180.0 / M_PI;
    yawDeg_ = yaw * 180.0 / M_PI;
    rollRate_ = rollRate;
    pitchRate_ = pitchRate;
    yawRate_ = yawRate;
    emit stateChanged();
}

void VehicleState::updateGlobalPosition(double lat, double lon, double altMsl,
                                        double relAlt, double vx, double vy, double vz,
                                        int headingDeg)
{
    lat_ = lat;
    lon_ = lon;
    altMsl_ = altMsl;
    relAlt_ = relAlt;
    vx_ = vx;
    vy_ = vy;
    vz_ = vz;
    headingDeg_ = headingDeg;
    if (groundspeed_ <= 0)
        groundspeed_ = std::sqrt(vx * vx + vy * vy);
    emit stateChanged();
}

void VehicleState::updateGps(uint8_t fix, uint8_t sats, uint16_t eph)
{
    gpsFix_ = fix;
    gpsSats_ = sats;
    gpsEph_ = eph;
    emit stateChanged();
}

void VehicleState::updateVfrHud(double airspeed, double groundspeed, double climb,
                                int headingDeg, int throttle)
{
    airspeed_ = airspeed;
    groundspeed_ = groundspeed;
    climb_ = climb;
    headingDeg_ = headingDeg;
    throttle_ = throttle;
    emit stateChanged();
}

void VehicleState::updateBattery(double voltage, double current, int remaining,
                                 int /*currentConsumed*/)
{
    batteryVoltage_ = voltage;
    batteryCurrent_ = current;
    batteryRemaining_ = remaining;
    emit stateChanged();
}

void VehicleState::updateSysStatus(int load, int voltageMv, int currentCa, int remainingPct)
{
    cpuLoad_ = load / 10;                       // 0-1000 -> 0-100
    if (voltageMv > 0 && voltageMv < 0xFFFF)
        batteryVoltage_ = voltageMv / 1000.0;
    if (currentCa >= 0)
        batteryCurrent_ = currentCa / 100.0;
    if (remainingPct >= 0)
        batteryRemaining_ = remainingPct;
    emit stateChanged();
}

void VehicleState::updateRadio(uint8_t rssi, uint8_t remRssi)
{
    radioRssi_ = rssi;
    radioRemRssi_ = remRssi;
    emit stateChanged();
}

void VehicleState::updateHome(double lat, double lon)
{
    hasHome_ = true;
    homeLat_ = lat;
    homeLon_ = lon;
    emit stateChanged();
}

void VehicleState::updateMissionCurrent(int seq, bool active)
{
    missionCurrent_ = seq;
    missionActive_ = active;
    emit missionChanged();
    emit stateChanged();
}

void VehicleState::updateMissionReached(int seq)
{
    missionReached_ = seq;
    emit missionChanged();
    emit stateChanged();
}

void VehicleState::setMissionUploaded(bool ok)
{
    missionUploaded_ = ok;
    emit missionChanged();
}

void VehicleState::setMissionResult(int result)
{
    missionResult_ = result;
    emit missionChanged();
}

// ---- 参数 (PARAM) ----
bool VehicleState::paramValue(const QString& name, float& value) const
{
    const auto it = params_.constFind(name);
    if (it == params_.constEnd())
        return false;
    value = it->value;
    return true;
}

uint8_t VehicleState::paramType(const QString& name) const
{
    const auto it = params_.constFind(name);
    return it == params_.constEnd() ? 0 : it->type;
}

bool VehicleState::paramDirty(const QString& name) const
{
    const auto it = params_.constFind(name);
    return it == params_.constEnd() ? false : it->dirty;
}

int VehicleState::paramIndex(const QString& name) const
{
    return paramOrder_.indexOf(name);
}

QString VehicleState::paramIdAt(int index) const
{
    if (index < 0 || index >= paramOrder_.size())
        return QString();
    return paramOrder_.at(index);
}

void VehicleState::updateParam(const QString& name, float value, uint8_t type,
                               int index, int count)
{
    ParamEntry& e = params_[name];
    const bool isNew = !e.dirty && (paramOrder_.indexOf(name) < 0);
    e.value = value;
    e.type = type;
    e.dirty = false;                    // 飞控回传 → 确认
    if (isNew) {
        paramOrder_.append(name);
        paramOrder_.sort(Qt::CaseInsensitive);
    }
    Q_UNUSED(index); Q_UNUSED(count);
    emit paramChanged(name, value);
    emit stateChanged();
}

void VehicleState::markParamDirty(const QString& name, bool dirty)
{
    auto it = params_.find(name);
    if (it == params_.end())
        return;
    it->dirty = dirty;
}

void VehicleState::clearParams()
{
    params_.clear();
    paramOrder_.clear();
    emit stateChanged();
}

void VehicleState::markOffline()
{
    if (!online_)
        return;
    online_ = false;
    emit connectionChanged(false);
    emit stateChanged();
}

} // namespace skygcs
