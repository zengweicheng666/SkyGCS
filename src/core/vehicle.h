#pragma once
// ============================================================================
// SkyGCS  飞行器状态聚合模型
// 由 MavlinkEndpoint 更新, UI 通过信号订阅刷新
// ============================================================================
#include <QObject>
#include <QDateTime>
#include <QString>
#include <cstdint>

namespace skygcs {

class VehicleState : public QObject {
    Q_OBJECT
public:
    explicit VehicleState(QObject* parent = nullptr);

    // ---- 连接/心跳 ----
    bool isOnline() const { return online_; }
    uint8_t sysid() const { return sysid_; }
    uint8_t compid() const { return compid_; }
    int heartbeatAgeMs() const;          // 距上次心跳的毫秒数
    qint64 lastHeartbeatMs() const { return lastHeartbeatMs_; }

    // ---- 识别 ----
    uint8_t type() const { return type_; }
    uint8_t autopilot() const { return autopilot_; }
    QString typeName() const;
    QString autopilotName() const;

    // ---- 模式/状态 ----
    bool armed() const { return armed_; }
    QString modeName() const;            // 解析 PX4 custom_mode
    uint8_t systemStatus() const { return systemStatus_; }
    QString systemStatusName() const;

    // ---- 姿态 ----
    double rollDeg() const { return rollDeg_; }
    double pitchDeg() const { return pitchDeg_; }
    double yawDeg() const { return yawDeg_; }
    double rollRate() const { return rollRate_; }
    double pitchRate() const { return pitchRate_; }
    double yawRate() const { return yawRate_; }

    // ---- 位置/速度 ----
    double lat() const { return lat_; }
    double lon() const { return lon_; }
    double altMsl() const { return altMsl_; }
    double relAlt() const { return relAlt_; }
    double vx() const { return vx_; }
    double vy() const { return vy_; }
    double vz() const { return vz_; }
    double groundspeed() const { return groundspeed_; }
    double airspeed() const { return airspeed_; }
    double climb() const { return climb_; }
    int headingDeg() const { return headingDeg_; }

    // ---- GPS ----
    uint8_t gpsFix() const { return gpsFix_; }
    uint8_t gpsSats() const { return gpsSats_; }
    uint16_t gpsEph() const { return gpsEph_; }
    QString gpsFixName() const;

    // ---- 电池/系统 ----
    double batteryVoltage() const { return batteryVoltage_; }
    double batteryCurrent() const { return batteryCurrent_; }
    int batteryRemaining() const { return batteryRemaining_; }
    int cpuLoad() const { return cpuLoad_; }

    // ---- 链路 ----
    int radioRssi() const { return radioRssi_; }
    int radioRemRssi() const { return radioRemRssi_; }

    // ---- 家点 ----
    bool hasHome() const { return hasHome_; }
    double homeLat() const { return homeLat_; }
    double homeLon() const { return homeLon_; }

    // ---- 任务 (Mission) ----
    int missionCurrent() const { return missionCurrent_; }   // 当前航点, 255=无
    int missionReached() const { return missionReached_; }   // 最后到达航点, -1=无
    bool missionActive() const { return missionActive_; }
    bool missionUploaded() const { return missionUploaded_; }
    int missionResult() const { return missionResult_; }     // MAV_MISSION_RESULT

    // ---- 由 endpoint 调用的更新接口 ----
    void updateHeartbeat(uint8_t sysid, uint8_t compid, uint8_t type, uint8_t autopilot,
                         uint8_t baseMode, uint32_t customMode, uint8_t systemStatus);
    void updateAttitude(double roll, double pitch, double yaw,
                        double rollRate, double pitchRate, double yawRate);
    void updateGlobalPosition(double lat, double lon, double altMsl, double relAlt,
                              double vx, double vy, double vz, int headingDeg);
    void updateGps(uint8_t fix, uint8_t sats, uint16_t eph);
    void updateVfrHud(double airspeed, double groundspeed, double climb, int headingDeg,
                      int throttle);
    void updateBattery(double voltage, double current, int remaining, int currentConsumed);
    void updateSysStatus(int load, int voltageMv, int currentCa, int remainingPct);
    void updateRadio(uint8_t rssi, uint8_t remRssi);
    void updateHome(double lat, double lon);
    void updateMissionCurrent(int seq, bool active);
    void updateMissionReached(int seq);
    void setMissionUploaded(bool ok);
    void setMissionResult(int result);
    void markOffline();

signals:
    void stateChanged();                  // 任何字段更新
    void connectionChanged(bool online);
    void missionChanged();                // 任务状态变化

private:
    bool online_ = false;
    uint8_t sysid_ = 0, compid_ = 0;
    uint8_t type_ = 0, autopilot_ = 0;
    uint8_t baseMode_ = 0;
    uint32_t customMode_ = 0;
    uint8_t systemStatus_ = 0;
    bool armed_ = false;
    qint64 lastHeartbeatMs_ = 0;

    double rollDeg_ = 0, pitchDeg_ = 0, yawDeg_ = 0;
    double rollRate_ = 0, pitchRate_ = 0, yawRate_ = 0;

    double lat_ = 0, lon_ = 0, altMsl_ = 0, relAlt_ = 0;
    double vx_ = 0, vy_ = 0, vz_ = 0;
    double groundspeed_ = 0, airspeed_ = 0, climb_ = 0;
    int headingDeg_ = 0;
    int throttle_ = 0;

    uint8_t gpsFix_ = 0, gpsSats_ = 0;
    uint16_t gpsEph_ = 0;

    double batteryVoltage_ = 0, batteryCurrent_ = 0;
    int batteryRemaining_ = -1;
    int cpuLoad_ = 0;

    int radioRssi_ = 0, radioRemRssi_ = 0;

    bool hasHome_ = false;
    double homeLat_ = 0, homeLon_ = 0;

    int missionCurrent_ = 255;
    int missionReached_ = -1;
    bool missionActive_ = false;
    bool missionUploaded_ = false;
    int missionResult_ = -1;
};

} // namespace skygcs
