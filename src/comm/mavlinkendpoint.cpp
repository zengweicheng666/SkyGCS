#include "mavlinkendpoint.h"

#include <QDateTime>

#include "../mavlink/mavlink_names.h"

namespace skygcs {

MavlinkEndpoint::MavlinkEndpoint(QObject* parent)
    : QObject(parent)
{
    heartbeatTimer_.setInterval(1000);
    connect(&heartbeatTimer_, &QTimer::timeout, this, &MavlinkEndpoint::onHeartbeatTimer);
    heartbeatTimer_.start();

    ackTimer_.setInterval(800);
    connect(&ackTimer_, &QTimer::timeout, this, &MavlinkEndpoint::onAckTimer);

    connect(&vehicle_, &VehicleState::connectionChanged, this, &MavlinkEndpoint::vehicleOnline);
}

// ---------------------------------------------------------------------------
// 链路管理
// ---------------------------------------------------------------------------
void MavlinkEndpoint::addLink(LinkInterface* link)
{
    if (!link || links_.contains(link))
        return;
    links_.append(link);
    auto* codec = new MavlinkCodec;   // 轻量值对象, 非 QObject
    codec->setLocalIds(255, 190);     // MAV_SYS_ID_GCS=255, MAV_COMP_ID_GCS=190
    codecs_.insert(link, codec);
    connect(link, &LinkInterface::bytesReceived, this, &MavlinkEndpoint::onLinkBytes);
    emit logMessage(QObject::tr("[引擎] 挂接链路: %1").arg(link->name()));
}

void MavlinkEndpoint::removeLink(LinkInterface* link)
{
    if (!link)
        return;
    links_.removeAll(link);
    if (auto* codec = codecs_.take(link)) {
        disconnect(link, &LinkInterface::bytesReceived, this, &MavlinkEndpoint::onLinkBytes);
        delete codec;
    }
    if (lastLink_ == link)
        lastLink_ = nullptr;
    emit logMessage(QObject::tr("[引擎] 摘除链路: %1").arg(link->name()));
}

// ---------------------------------------------------------------------------
// 收包
// ---------------------------------------------------------------------------
void MavlinkEndpoint::onLinkBytes(const QByteArray& data)
{
    auto* link = qobject_cast<LinkInterface*>(sender());
    if (!link)
        return;
    lastLink_ = link;
    MavlinkCodec* codec = codecs_.value(link);
    if (!codec)
        return;

    MavMessage msg;
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(data.constData());
    size_t len = static_cast<size_t>(data.size());
    ParseStatus st = codec->decodeNext(raw, len, msg);
    while (st == ParseStatus::Complete) {
        handleMessage(msg);
        st = codec->decodeNext(nullptr, 0, msg);
    }
}

void MavlinkEndpoint::handleMessage(const MavMessage& msg)
{
    emit messageReceived(msg);
    switch (msg.msgid) {
    case MAV_MSG_ID_HEARTBEAT: handleHeartbeat(msg); break;
    case MAV_MSG_ID_ATTITUDE: handleAttitude(msg); break;
    case MAV_MSG_ID_GLOBAL_POSITION_INT: handleGlobalPosition(msg); break;
    case MAV_MSG_ID_GPS_RAW_INT: handleGpsRaw(msg); break;
    case MAV_MSG_ID_VFR_HUD: handleVfrHud(msg); break;
    case MAV_MSG_ID_BATTERY_STATUS: handleBattery(msg); break;
    case MAV_MSG_ID_SYS_STATUS: handleSysStatus(msg); break;
    case MAV_MSG_ID_STATUSTEXT: handleStatustext(msg); break;
    case MAV_MSG_ID_HOME_POSITION: handleHome(msg); break;
    case MAV_MSG_ID_RADIO_STATUS: handleRadio(msg); break;
    case MAV_MSG_ID_COMMAND_ACK: handleCommandAck(msg); break;
    case MAV_MSG_ID_PING: handlePing(msg); break;
    case MAV_MSG_ID_MISSION_REQUEST_INT: handleMissionRequestInt(msg); break;
    case MAV_MSG_ID_MISSION_ACK: handleMissionAck(msg); break;
    case MAV_MSG_ID_MISSION_CURRENT: handleMissionCurrent(msg); break;
    case MAV_MSG_ID_MISSION_ITEM_REACHED: handleMissionItemReached(msg); break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
// 遥测解码
// ---------------------------------------------------------------------------
void MavlinkEndpoint::handleHeartbeat(const MavMessage& msg)
{
    HeartbeatMsg m;
    if (!unpackHeartbeat(msg, m))
        return;
    targetSysid_ = msg.sysid;
    targetCompid_ = msg.compid;
    vehicle_.updateHeartbeat(msg.sysid, msg.compid, m.type, m.autopilot,
                             m.base_mode, m.custom_mode, m.system_status);
}

void MavlinkEndpoint::handleAttitude(const MavMessage& msg)
{
    AttitudeMsg m;
    if (unpackAttitude(msg, m))
        vehicle_.updateAttitude(m.roll, m.pitch, m.yaw, m.rollspeed, m.pitchspeed, m.yawspeed);
}

void MavlinkEndpoint::handleGlobalPosition(const MavMessage& msg)
{
    GlobalPositionIntMsg m;
    if (!unpackGlobalPositionInt(msg, m))
        return;
    vehicle_.updateGlobalPosition(m.lat / 1e7, m.lon / 1e7, m.alt / 1e3, m.relative_alt / 1e3,
                                  m.vx / 100.0, m.vy / 100.0, m.vz / 100.0, m.hdg / 100);
}

void MavlinkEndpoint::handleGpsRaw(const MavMessage& msg)
{
    GpsRawIntMsg m;
    if (unpackGpsRawInt(msg, m))
        vehicle_.updateGps(m.fix_type, m.satellites_visible, m.eph);
}

void MavlinkEndpoint::handleVfrHud(const MavMessage& msg)
{
    VfrHudMsg m;
    if (unpackVfrHud(msg, m))
        vehicle_.updateVfrHud(m.airspeed, m.groundspeed, m.climb, m.heading, m.throttle);
}

void MavlinkEndpoint::handleBattery(const MavMessage& msg)
{
    BatteryStatusMsg m;
    if (unpackBatteryStatus(msg, m))
        vehicle_.updateBattery(m.voltages[0] / 1000.0, m.current_battery / 100.0,
                               m.battery_remaining, m.current_consumed);
}

void MavlinkEndpoint::handleSysStatus(const MavMessage& msg)
{
    SysStatusMsg m;
    if (unpackSysStatus(msg, m))
        vehicle_.updateSysStatus(m.load, m.voltage_battery, m.current_battery,
                                 m.battery_remaining);
}

void MavlinkEndpoint::handleStatustext(const MavMessage& msg)
{
    StatustextMsg m;
    if (!unpackStatustext(msg, m))
        return;
    // 截断以 NUL 结尾的文本
    QString text = QString::fromLatin1(m.text);
    const int nul = text.indexOf(QChar('\0'));
    if (nul >= 0)
        text.truncate(nul);
    emit statustext(m.severity, text);
}

void MavlinkEndpoint::handleHome(const MavMessage& msg)
{
    HomePositionMsg m;
    if (unpackHomePosition(msg, m))
        vehicle_.updateHome(m.latitude / 1e7, m.longitude / 1e7);
}

void MavlinkEndpoint::handleRadio(const MavMessage& msg)
{
    RadioStatusMsg m;
    if (unpackRadioStatus(msg, m))
        vehicle_.updateRadio(m.rssi, m.remrssi);
}

void MavlinkEndpoint::handleCommandAck(const MavMessage& msg)
{
    CommandAckMsg ack;
    if (!unpackCommandAck(msg, ack))
        return;
    emit commandAck(ack.command, ack.result, ack.progress, ack.result_param2);
    // 匹配待确认指令
    if (pending_.command == ack.command) {
        ackTimer_.stop();
        pending_.command = 0;
    }
}

void MavlinkEndpoint::handlePing(const MavMessage& msg)
{
    PingMsg m;
    if (!unpackPing(msg, m))
        return;
    if (m.target_system == 0 && m.target_component == 0) {
        // 收到 ping 请求 → 回 ping (带原 seq)
        m.target_system = msg.sysid;
        m.target_component = msg.compid;
        MavMessage reply;
        reply.msgid = MAV_MSG_ID_PING;
        reply.sysid = 255;
        reply.compid = 190;
        if (MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_PING), &m, reply))
            sendFrame(lastLink_, reply);
    }
}

// ---------------------------------------------------------------------------
// 任务 (Mission)
// ---------------------------------------------------------------------------
void MavlinkEndpoint::handleMissionRequestInt(const MavMessage& msg)
{
    MissionRequestIntMsg req;
    if (!unpackMissionRequestInt(msg, req))
        return;
    if (!missionUpload_.active || !lastLink_)
        return;
    // 飞控请求指定航点 → 回发对应 ITEM
    if (req.seq >= missionUpload_.items.size()) {
        emit logMessage(QObject::tr("[任务] 飞控请求越界航点 %1, 中止上传").arg(req.seq));
        missionUpload_.active = false;
        vehicle_.setMissionUploaded(false);
        return;
    }
    const MissionItem& it = missionUpload_.items[req.seq];
    MavMessage out;
    out.msgid = MAV_MSG_ID_MISSION_ITEM_INT;
    out.sysid = 255;
    out.compid = 190;
    if (packMissionItemInt(static_cast<uint16_t>(req.seq), it.frame, it.command,
                           it.x, it.y, it.z, targetSysid_, targetCompid_, out,
                           it.p1, it.p2, it.p3, it.p4)) {
        sendFrame(lastLink_, out);
        emit logMessage(QObject::tr("[任务] 已发送航点 %1/%2")
                        .arg(req.seq + 1).arg(missionUpload_.items.size()));
    }
}

void MavlinkEndpoint::handleMissionAck(const MavMessage& msg)
{
    MissionAckMsg ack;
    if (!unpackMissionAck(msg, ack))
        return;
    missionUpload_.active = false;
    vehicle_.setMissionUploaded(ack.type == mav::MISSION_ACCEPTED);
    vehicle_.setMissionResult(ack.type);
    emit logMessage(QObject::tr("[任务] 上传完成, 结果=%1")
                    .arg(mav::missionResultName(ack.type)));
}

void MavlinkEndpoint::handleMissionCurrent(const MavMessage& msg)
{
    MissionCurrentMsg m;
    if (!unpackMissionCurrent(msg, m))
        return;
    vehicle_.updateMissionCurrent(m.seq, m.seq != 255);
}

void MavlinkEndpoint::handleMissionItemReached(const MavMessage& msg)
{
    MissionItemReachedMsg m;
    if (!unpackMissionItemReached(msg, m))
        return;
    vehicle_.updateMissionReached(m.seq);
}

// ---------------------------------------------------------------------------
// 周期心跳
// ---------------------------------------------------------------------------
void MavlinkEndpoint::onHeartbeatTimer()
{
    if (links_.isEmpty())
        return;
    MavMessage hb;
    if (packHeartbeat(mav::TYPE_GCS, mav::AUTOPILOT_INVALID, 0, 0,
                      mav::STATE_ACTIVE, hb)) {
        hb.sysid = 255;
        hb.compid = 190;
        for (LinkInterface* link : links_)
            sendFrame(link, hb);
    }
    // 连接超时判定 (5s 无心跳)
    if (vehicle_.isOnline() && vehicle_.heartbeatAgeMs() > 5000) {
        emit logMessage(QObject::tr("[引擎] 飞行器心跳超时, 判定离线"));
        vehicle_.markOffline();
    }
}

// ---------------------------------------------------------------------------
// 发送
// ---------------------------------------------------------------------------
void MavlinkEndpoint::sendFrame(LinkInterface* link, const MavMessage& msg)
{
    if (!link || !link->isOpen())
        return;
    MavlinkCodec* codec = codecs_.value(link);
    if (!codec)
        return;
    // 统一以 GCS 身份发送
    MavMessage out = msg;
    out.sysid = 255;
    out.compid = 190;
    const std::vector<uint8_t> frameVec = codec->encodeFrame(out);
    link->sendBytes(QByteArray(reinterpret_cast<const char*>(frameVec.data()),
                               static_cast<int>(frameVec.size())));
}

bool MavlinkEndpoint::sendMessage(const MavMessage& msg)
{
    if (links_.isEmpty())
        return false;
    for (LinkInterface* link : links_)
        sendFrame(link, msg);
    return true;
}

bool MavlinkEndpoint::sendCommandLong(uint16_t command, float p1, float p2, float p3,
                                      float p4, float p5, float p6, float p7)
{
    CommandLongMsg m{};
    m.param1 = p1; m.param2 = p2; m.param3 = p3; m.param4 = p4;
    m.param5 = p5; m.param6 = p6; m.param7 = p7;
    m.command = command;
    m.target_system = targetSysid_;
    m.target_component = targetCompid_;
    m.confirmation = 0;

    MavMessage msg;
    if (!MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_COMMAND_LONG), &m, msg))
        return false;
    if (!sendMessage(msg))
        return false;

    pending_.command = command;
    pending_.params = m;
    pending_.retries = 0;
    pending_.sentAtMs = QDateTime::currentMSecsSinceEpoch();
    ackTimer_.start();
    return true;
}

bool MavlinkEndpoint::sendSetMode(uint32_t mainMode, uint32_t subMode)
{
    SetModeMsg m{};
    m.custom_mode = mav::px4CustomModeEncode(mainMode, subMode);
    m.target_system = targetSysid_;
    m.base_mode = mav::MODE_FLAG_CUSTOM_MODE_ENABLED;

    MavMessage msg;
    if (!MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_SET_MODE), &m, msg))
        return false;
    return sendMessage(msg);
}

bool MavlinkEndpoint::armDisarm(bool arm)
{
    // MAV_CMD_COMPONENT_ARM_DISARM: param1=1 解锁 / 0 上锁
    return sendCommandLong(mav::CMD_COMPONENT_ARM_DISARM, arm ? 1.0f : 0.0f);
}

bool MavlinkEndpoint::takeoff(float altM)
{
    return sendCommandLong(mav::CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altM);
}

bool MavlinkEndpoint::land()
{
    return sendCommandLong(mav::CMD_NAV_LAND);
}

bool MavlinkEndpoint::rtl()
{
    return sendCommandLong(mav::CMD_NAV_RETURN_TO_LAUNCH);
}

bool MavlinkEndpoint::requestHome()
{
    return sendCommandLong(mav::CMD_GET_HOME_POSITION);
}

bool MavlinkEndpoint::sendPing()
{
    PingMsg m{};
    m.time_usec = static_cast<uint64_t>(QDateTime::currentMSecsSinceEpoch() * 1000);
    m.seq = static_cast<uint32_t>(QDateTime::currentMSecsSinceEpoch() & 0xFFFFFFFF);
    m.target_system = 0;     // 广播
    m.target_component = 0;
    MavMessage msg;
    if (!MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_PING), &m, msg))
        return false;
    return sendMessage(msg);
}

// ---------------------------------------------------------------------------
// 任务上传 (异步状态机: COUNT → REQUEST_INT(seq) → ITEM(seq) → … → ACK)
// ---------------------------------------------------------------------------
bool MavlinkEndpoint::uploadMission(const QVector<MissionItem>& items)
{
    if (items.isEmpty() || items.size() > 0xFFFF) {
        emit logMessage(QObject::tr("[任务] 航点数量非法: %1").arg(items.size()));
        return false;
    }
    missionUpload_.items = items;
    missionUpload_.nextSeq = 0;
    missionUpload_.active = true;
    missionUpload_.startedAtMs = QDateTime::currentMSecsSinceEpoch();
    vehicle_.setMissionUploaded(false);

    MavMessage out;
    out.msgid = MAV_MSG_ID_MISSION_COUNT;
    out.sysid = 255;
    out.compid = 190;
    if (!packMissionCount(static_cast<uint16_t>(items.size()), targetSysid_, targetCompid_, out))
        return false;
    if (!sendMessage(out))
        return false;
    emit logMessage(QObject::tr("[任务] 开始上传 %1 个航点...").arg(items.size()));
    return true;
}

bool MavlinkEndpoint::startMission()
{
    return sendCommandLong(mav::CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0);
}

bool MavlinkEndpoint::abortMissionUpload()
{
    missionUpload_.active = false;
    vehicle_.setMissionUploaded(false);
    return true;
}

void MavlinkEndpoint::onAckTimer()
{
    if (pending_.command == 0)
        return;
    const qint64 elapsed = QDateTime::currentMSecsSinceEpoch() - pending_.sentAtMs;
    if (elapsed > 4000) {
        emit logMessage(QObject::tr("[引擎] 指令 0x%1 ACK 超时(4s), 放弃重发")
                            .arg(pending_.command, 4, 16, QChar('0')));
        pending_.command = 0;
        ackTimer_.stop();
        return;
    }
    if (pending_.retries >= 3) {
        emit logMessage(QObject::tr("[引擎] 指令 0x%1 重发 3 次仍未确认")
                            .arg(pending_.command, 4, 16, QChar('0')));
        pending_.command = 0;
        ackTimer_.stop();
        return;
    }
    ++pending_.retries;
    emit logMessage(QObject::tr("[引擎] 重发指令 0x%1 (%2/3)")
                        .arg(pending_.command, 4, 16, QChar('0')).arg(pending_.retries));
    // 用保留的原始参数重发
    MavMessage msg;
    if (MavlinkCodec::pack(*MavlinkCodec::findDef(MAV_MSG_ID_COMMAND_LONG),
                           &pending_.params, msg)) {
        sendMessage(msg);
        pending_.sentAtMs = QDateTime::currentMSecsSinceEpoch();
    } else {
        pending_.command = 0;
        ackTimer_.stop();
    }
}

} // namespace skygcs
