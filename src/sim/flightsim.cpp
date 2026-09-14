#include "flightsim.h"

#include <QDateTime>
#include <QHostAddress>
#include <algorithm>
#include <cmath>

#include "../mavlink/mavlink_messages.h"

namespace skygcs {

namespace {
constexpr double G = 9.81;
constexpr double MAX_CLIMB = 3.0;
constexpr double MAX_SPEED = 5.0;
constexpr double MAX_TILT = 0.35;    // rad (~20deg)
} // namespace

FlightSim::FlightSim(QObject* parent)
    : QObject(parent)
{
    timer_.setInterval(50);   // 20 Hz
    connect(&timer_, &QTimer::timeout, this, &FlightSim::onTick);
    connect(&udp_, &QUdpSocket::readyRead, this, &FlightSim::onUdpReady);
    codec_.setLocalIds(1, 1);   // 模拟 PX4: sysid=1, compid=1
}

void FlightSim::setHome(double lat, double lon, double altMsl)
{
    homeLat_ = lat;
    homeLon_ = lon;
    homeAltMsl_ = altMsl;
}

bool FlightSim::startInject()
{
    mode_ = QStringLiteral("内置仿真(注入)");
    startMs_ = QDateTime::currentMSecsSinceEpoch();
    running_ = true;
    timer_.start();
    emitStatustext(mav::SEVERITY_INFO, "SkyGCS 内置仿真启动");
    return true;
}

bool FlightSim::startUdpServer(quint16 gcsPort)
{
    gcsPort_ = gcsPort;
    if (!udp_.bind(QHostAddress::AnyIPv4, 0))   // 绑定临时端口发送/接收
        return false;
    mode_ = QStringLiteral("内置仿真(UDP服务 :%1)").arg(gcsPort_);
    startMs_ = QDateTime::currentMSecsSinceEpoch();
    running_ = true;
    timer_.start();
    emitStatustext(mav::SEVERITY_INFO,
                   QStringLiteral("SkyGCS 仿真 UDP 服务启动 (遥测->%1:%2)")
                       .arg(gcsAddr_.toString())
                       .arg(gcsPort_));
    return true;
}

void FlightSim::stop()
{
    timer_.stop();
    if (udp_.state() != QAbstractSocket::UnconnectedState)
        udp_.close();
    running_ = false;
}

// ---------------------------------------------------------------------------
// 发送
// ---------------------------------------------------------------------------
void FlightSim::sendMessage(const MavMessage& msg)
{
    if (mode_.startsWith(QStringLiteral("内置仿真(注入)"))) {
        emit messageReady(msg);
    } else {
        MavMessage out = msg;
        out.sysid = 1;
        out.compid = 1;
        const std::vector<uint8_t> frame = codec_.encodeFrame(out);
        udp_.writeDatagram(reinterpret_cast<const char*>(frame.data()),
                           static_cast<qint64>(frame.size()), gcsAddr_, gcsPort_);
    }
}

void FlightSim::encodeAndSend(uint32_t msgid, const void* structPtr)
{
    const MsgDef* def = MavlinkCodec::findDef(msgid);
    if (!def)
        return;
    MavMessage msg;
    if (MavlinkCodec::pack(*def, structPtr, msg))
        sendMessage(msg);
}

void FlightSim::emitStatustext(int severity, const QString& text)
{
    StatustextMsg m{};
    m.severity = static_cast<uint8_t>(severity);
    const QByteArray utf8 = text.toUtf8();
    std::memset(m.text, 0, sizeof(m.text));
    std::memcpy(m.text, utf8.constData(), std::min<size_t>(sizeof(m.text) - 1, utf8.size()));
    encodeAndSend(MAV_MSG_ID_STATUSTEXT, &m);
}

// ---------------------------------------------------------------------------
// 指令处理
// ---------------------------------------------------------------------------
void FlightSim::onUdpReady()
{
    while (udp_.hasPendingDatagrams()) {
        QByteArray dg;
        dg.resize(static_cast<int>(udp_.pendingDatagramSize()));
        QHostAddress from;
        quint16 fromPort = 0;
        udp_.readDatagram(dg.data(), dg.size(), &from, &fromPort);
        if (fromPort != 0) {
            gcsAddr_ = from;   // 记住地面站地址, 后续遥测发往对端
        }
        MavMessage msg;
        ParseStatus st = codec_.decodeNext(
            reinterpret_cast<const uint8_t*>(dg.constData()),
            static_cast<size_t>(dg.size()), msg);
        while (st == ParseStatus::Complete) {
            handleCommand(msg);
            st = codec_.decodeNext(nullptr, 0, msg);
        }
    }
}

void FlightSim::handleCommand(const MavMessage& msg)
{
    switch (msg.msgid) {
    case MAV_MSG_ID_COMMAND_LONG: {
        CommandLongMsg c;
        const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_COMMAND_LONG);
        if (def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &c))
            processCommandLong(c);
        break;
    }
    case MAV_MSG_ID_SET_MODE: {
        SetModeMsg m;
        const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_SET_MODE);
        if (def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &m)) {
            setMode(mav::px4CustomModeMain(m.custom_mode), mav::px4CustomModeSub(m.custom_mode));
            emitStatustext(mav::SEVERITY_INFO, "模式切换: " + modeName());
        }
        break;
    }
    case MAV_MSG_ID_PING: {
        PingMsg m;
        const MsgDef* def = MavlinkCodec::findDef(MAV_MSG_ID_PING);
        if (def && MavlinkCodec::unpack(*def, msg.data(), msg.size(), &m)) {
            m.target_system = msg.sysid;
            m.target_component = msg.compid;
            encodeAndSend(MAV_MSG_ID_PING, &m);
        }
        break;
    }
    default:
        break;
    }
}

void FlightSim::processCommandLong(const CommandLongMsg& c)
{
    CommandAckMsg ack{};
    ack.command = c.command;
    ack.target_system = c.target_system;
    ack.target_component = c.target_component;

    switch (c.command) {
    case mav::CMD_COMPONENT_ARM_DISARM:
        if (c.param1 > 0.5f && !armed_) {
            armed_ = true;
            emitStatustext(mav::SEVERITY_INFO, "电机解锁 (ARMED)");
            if (mainMode_ == mav::PX4_MODE_MANUAL || mainMode_ == mav::PX4_MODE_POSCTL)
                setMode(mav::PX4_MODE_POSCTL, 0);
        } else if (c.param1 < 0.5f && armed_) {
            armed_ = false;
            velN_[0] = velN_[1] = 0;
            emitStatustext(mav::SEVERITY_WARNING, "电机上锁 (DISARMED)");
        }
        ack.result = mav::RESULT_ACCEPTED;
        break;
    case mav::CMD_NAV_TAKEOFF:
        if (!armed_) {
            ack.result = mav::RESULT_DENIED;
            emitStatustext(mav::SEVERITY_ERROR, "起飞被拒: 未解锁");
        } else {
            targetAlt_ = std::max(1.0, static_cast<double>(c.param7));
            setMode(mav::PX4_MODE_AUTO, mav::PX4_AUTO_TAKEOFF);
            emitStatustext(mav::SEVERITY_INFO,
                           QStringLiteral("起飞指令: 目标高度 %1 m").arg(targetAlt_));
            ack.result = mav::RESULT_ACCEPTED;
        }
        break;
    case mav::CMD_NAV_LAND:
        setMode(mav::PX4_MODE_AUTO, mav::PX4_AUTO_LAND);
        emitStatustext(mav::SEVERITY_INFO, "降落指令");
        ack.result = mav::RESULT_ACCEPTED;
        break;
    case mav::CMD_NAV_RETURN_TO_LAUNCH:
        setMode(mav::PX4_MODE_AUTO, mav::PX4_AUTO_RTL);
        emitStatustext(mav::SEVERITY_INFO, "返航指令 (RTL)");
        ack.result = mav::RESULT_ACCEPTED;
        break;
    case mav::CMD_GET_HOME_POSITION:
        ack.result = mav::RESULT_ACCEPTED;
        break;
    default:
        ack.result = mav::RESULT_UNSUPPORTED;
        break;
    }
    encodeAndSend(MAV_MSG_ID_COMMAND_ACK, &ack);
}

void FlightSim::setMode(uint32_t mainMode, uint32_t subMode)
{
    mainMode_ = mainMode;
    subMode_ = subMode;
    if (mainMode_ == mav::PX4_MODE_AUTO && subMode_ == mav::PX4_AUTO_LAND) {
        // 降落结束自动上锁
        if (posN_[2] > -0.3) {
            armed_ = false;
        }
    }
}

QString FlightSim::modeName() const
{
    switch (mainMode_) {
    case mav::PX4_MODE_MANUAL: return "Manual";
    case mav::PX4_MODE_POSCTL: return "PosCtl";
    case mav::PX4_MODE_AUTO:
        switch (subMode_) {
        case mav::PX4_AUTO_TAKEOFF: return "Auto.Takeoff";
        case mav::PX4_AUTO_LOITER: return "Auto.Loiter";
        case mav::PX4_AUTO_MISSION: return "Auto.Mission";
        case mav::PX4_AUTO_RTL: return "Auto.RTL";
        case mav::PX4_AUTO_LAND: return "Auto.Land";
        default: return "Auto";
        }
    case mav::PX4_MODE_OFFBOARD: return "Offboard";
    default: return "Unknown";
    }
}

// ---------------------------------------------------------------------------
// 20Hz 物理推进 + 遥测发送
// ---------------------------------------------------------------------------
void FlightSim::onTick()
{
    const double dt = 0.05;
    ++tick_;

    // ---- 物理模型 (简化四旋翼) ----
    double ax = 0, ay = 0, az = 0;   // NED 加速度 (净)
    double pitchTarget = 0, rollTarget = 0;
    double thrust = 0;               // 垂直推力产生的向上加速度 (NED 负向)

    if (armed_) {
        // 垂直: 高度 P 控制 + 速度阻尼
        double targetZ = -targetAlt_;   // NED: 高度向上为负
        if (mainMode_ == mav::PX4_MODE_AUTO && subMode_ == mav::PX4_AUTO_LAND)
            targetZ = -0.02;
        if (mainMode_ == mav::PX4_MODE_AUTO && subMode_ == mav::PX4_AUTO_RTL)
            targetZ = std::min(-targetAlt_, posN_[2] - 1.0);
        const double desiredVel = std::clamp((targetZ - posN_[2]) * 0.6, -MAX_CLIMB, MAX_CLIMB);
        // 推力修正: 当前速度高于期望(过快上升/下降)则减推力; desiredVel 向上为负
        thrust = G + std::clamp((velN_[2] - desiredVel) * 1.2, -4.0, 4.0);
        az = G - thrust;   // NED: 重力 +z, 推力 -z

        // 水平: 回中漂移抑制
        ax = -velN_[0] * 0.5;
        ay = -velN_[1] * 0.5;

        // 任务水平移动: RTL 飞回家点
        if (mainMode_ == mav::PX4_MODE_AUTO && subMode_ == mav::PX4_AUTO_RTL) {
            const double dist = std::sqrt(posN_[0] * posN_[0] + posN_[1] * posN_[1]);
            if (dist > 2.0) {
                ax = -posN_[0] / dist * 2.0;
                ay = -posN_[1] / dist * 2.0;
            } else if (dist <= 2.0 && posN_[2] > -0.5) {
                // 到家后降落
                setMode(mav::PX4_MODE_AUTO, mav::PX4_AUTO_LAND);
                emitStatustext(mav::SEVERITY_INFO, "已到达家点, 自动降落");
            }
        }

        // 姿态跟随加速度方向 (简化)
        pitchTarget = std::clamp(ax / G * 0.5, -MAX_TILT, MAX_TILT);
        rollTarget = std::clamp(-ay / G * 0.5, -MAX_TILT, MAX_TILT);
        att_[0] += (rollTarget - att_[0]) * 2.0 * dt;
        att_[1] += (pitchTarget - att_[1]) * 2.0 * dt;
    } else {
        thrust = 0;
        az = G - thrust;                                             // 自由下落, 地面约束
        att_[0] *= 0.9;
        att_[1] *= 0.9;
    }

    velN_[0] += ax * dt;
    velN_[1] += ay * dt;
    velN_[2] += az * dt;   // az 已是净加速度 (重力+推力), 勿再减 G
    // 地面约束
    if (posN_[2] >= 0 && velN_[2] > 0) {
        posN_[2] = 0;
        velN_[2] = 0;
    }
    posN_[0] += velN_[0] * dt;
    posN_[1] += velN_[1] * dt;
    posN_[2] += velN_[2] * dt;
    // 降落触地
    if (posN_[2] > -0.05 && velN_[2] > -0.05) {
        posN_[2] = 0;
        velN_[2] = 0;
        if (mainMode_ == mav::PX4_MODE_AUTO && subMode_ == mav::PX4_AUTO_LAND && armed_) {
            armed_ = false;
            emitStatustext(mav::SEVERITY_INFO, "降落完成, 自动上锁");
        }
    }

    // 电池消耗
    if (armed_)
        battery_ = std::max(0.0, battery_ - 0.03);   // 约 0.6%/min
    if (battery_ < 25.0 && battery_ > 24.9)
        emitStatustext(mav::SEVERITY_WARNING, "电池电量低 (25%)");

    // ---- 遥测发送 ----
    const qint64 bootMs = QDateTime::currentMSecsSinceEpoch() - startMs_;
    const double lat = homeLat_ + posN_[0] / 111320.0;
    const double lon = homeLon_ + posN_[1] / (111320.0 * std::cos(homeLat_ * M_PI / 180.0));

    // HEARTBEAT 1Hz
    if (tick_ % 20 == 0) {
        HeartbeatMsg hb{};
        hb.type = mav::TYPE_QUADROTOR;
        hb.autopilot = mav::AUTOPILOT_PX4;
        hb.base_mode = (armed_ ? mav::MODE_FLAG_SAFETY_ARMED : 0)
                     | mav::MODE_FLAG_CUSTOM_MODE_ENABLED;
        hb.custom_mode = mav::px4CustomModeEncode(mainMode_, subMode_);
        hb.system_status = armed_ ? mav::STATE_ACTIVE : mav::STATE_STANDBY;
        hb.mavlink_version = 3;
        encodeAndSend(MAV_MSG_ID_HEARTBEAT, &hb);
    }
    // ATTITUDE 20Hz
    {
        AttitudeMsg a{};
        a.time_boot_ms = static_cast<uint32_t>(bootMs);
        a.roll = att_[0];
        a.pitch = att_[1];
        a.yaw = att_[2];
        a.rollspeed = rates_[0];
        a.pitchspeed = rates_[1];
        a.yawspeed = rates_[2];
        encodeAndSend(MAV_MSG_ID_ATTITUDE, &a);
    }
    // GLOBAL_POSITION_INT 5Hz
    if (tick_ % 4 == 0) {
        GlobalPositionIntMsg p{};
        p.time_boot_ms = static_cast<uint32_t>(bootMs);
        p.lat = static_cast<int32_t>(lat * 1e7);
        p.lon = static_cast<int32_t>(lon * 1e7);
        p.alt = static_cast<int32_t>((homeAltMsl_ - posN_[2]) * 1e3);
        p.relative_alt = static_cast<int32_t>((-posN_[2]) * 1e3);
        p.vx = static_cast<int16_t>(velN_[0] * 100);
        p.vy = static_cast<int16_t>(velN_[1] * 100);
        p.vz = static_cast<int16_t>(velN_[2] * 100);
        const int hdgDeg = static_cast<int>(std::fmod(att_[2] * 180.0 / M_PI + 360.0, 360.0));
        p.hdg = static_cast<uint16_t>(hdgDeg * 100);
        encodeAndSend(MAV_MSG_ID_GLOBAL_POSITION_INT, &p);
    }
    // GPS_RAW_INT 2Hz
    if (tick_ % 10 == 0) {
        GpsRawIntMsg g{};
        g.time_usec = static_cast<uint64_t>(QDateTime::currentMSecsSinceEpoch() * 1000);
        g.lat = static_cast<int32_t>(lat * 1e7);
        g.lon = static_cast<int32_t>(lon * 1e7);
        g.alt = static_cast<int32_t>((homeAltMsl_ - posN_[2]) * 1e3);
        g.eph = 80;
        g.epv = 120;
        g.vel = static_cast<uint16_t>(std::sqrt(velN_[0] * velN_[0] + velN_[1] * velN_[1]) * 100);
        g.cog = 0;
        g.fix_type = 3;
        g.satellites_visible = 16;
        encodeAndSend(MAV_MSG_ID_GPS_RAW_INT, &g);
    }
    // VFR_HUD 5Hz
    if (tick_ % 4 == 0) {
        VfrHudMsg v{};
        v.airspeed = static_cast<float>(std::sqrt(velN_[0] * velN_[0] + velN_[1] * velN_[1]));
        v.groundspeed = v.airspeed;
        v.heading = static_cast<int16_t>(
            static_cast<int>(att_[2] * 180.0 / M_PI + 360.0) % 360);
        v.throttle = static_cast<uint16_t>(armed_ ? 55 : 0);
        v.alt = static_cast<float>(homeAltMsl_ - posN_[2]);
        v.climb = static_cast<float>(-velN_[2]);
        encodeAndSend(MAV_MSG_ID_VFR_HUD, &v);
    }
    // SYS_STATUS + BATTERY_STATUS 1Hz
    if (tick_ % 20 == 0) {
        SysStatusMsg s{};
        s.load = 320;
        s.voltage_battery = static_cast<uint16_t>((14.0 + battery_ / 100.0 * 2.8) * 1000);
        s.current_battery = static_cast<int16_t>(armed_ ? 850 : 80);
        s.battery_remaining = static_cast<int8_t>(battery_);
        s.onboard_control_sensors_present = 0xFFFFFFFF;
        s.onboard_control_sensors_enabled = 0xFFFFFFFF;
        s.onboard_control_sensors_health = 0xFFFFFFF0;
        encodeAndSend(MAV_MSG_ID_SYS_STATUS, &s);

        BatteryStatusMsg b{};
        b.id = 0;
        b.voltages[0] = static_cast<uint16_t>((14.0 + battery_ / 100.0 * 2.8) * 1000);
        b.current_battery = static_cast<int16_t>(armed_ ? 850 : 80);
        b.battery_remaining = static_cast<int8_t>(battery_);
        b.current_consumed = static_cast<int32_t>((100.0 - battery_) * 25);
        encodeAndSend(MAV_MSG_ID_BATTERY_STATUS, &b);
    }
    // HOME_POSITION 首帧后 1s
    if (tick_ == 20) {
        HomePositionMsg h{};
        h.latitude = static_cast<int32_t>(homeLat_ * 1e7);
        h.longitude = static_cast<int32_t>(homeLon_ * 1e7);
        h.altitude = static_cast<int32_t>(homeAltMsl_ * 1e3);
        encodeAndSend(MAV_MSG_ID_HOME_POSITION, &h);
    }
}

} // namespace skygcs
