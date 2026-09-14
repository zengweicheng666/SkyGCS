#include "flightlog.h"

#include <QDateTime>
#include <QFileInfo>
#include <QDir>
#include <cmath>

#include "../mavlink/mavlink_types.h"
#include "vehicle.h"

namespace skygcs {

// ============================================================================
// 记录器
// ============================================================================
FlightLogRecorder::FlightLogRecorder(VehicleState* vehicle, QObject* parent)
    : QObject(parent)
    , vehicle_(vehicle)
{
    timer_.setInterval(500);
    connect(&timer_, &QTimer::timeout, this, &FlightLogRecorder::onSample);
}

bool FlightLogRecorder::start(const QString& filePath)
{
    if (recording_)
        stop();
    if (filePath.isEmpty())
        return false;
    file_.setFileName(filePath);
    if (!file_.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
        return false;
    out_.setDevice(&file_);
    writeHeader();
    recording_ = true;
    filePath_ = filePath;
    samples_ = 0;
    timer_.start();
    emit logMessage(QObject::tr("[日志] 开始记录遥测 → %1").arg(filePath));
    return true;
}

void FlightLogRecorder::stop()
{
    if (!recording_)
        return;
    timer_.stop();
    out_.flush();
    file_.close();
    recording_ = false;
    emit logMessage(QObject::tr("[日志] 记录停止, 共 %1 个采样点").arg(samples_));
}

void FlightLogRecorder::writeHeader()
{
    out_ << "t_ms,armed,mode_main,mode_sub,lat,lon,alt_msl,rel_alt,"
            "climb,groundspeed,heading,battery,mission_cur,mission_reached\n";
}

void FlightLogRecorder::onSample()
{
    if (!vehicle_)
        return;
    const VehicleState& v = *vehicle_;
    const uint32_t main = mav::px4CustomModeMain(v.customMode());
    const uint32_t sub = mav::px4CustomModeSub(v.customMode());

    out_ << QDateTime::currentMSecsSinceEpoch() << ','
         << (v.armed() ? 1 : 0) << ','
         << main << ',' << sub << ','
         << QString::number(v.lat(), 'f', 7) << ','
         << QString::number(v.lon(), 'f', 7) << ','
         << QString::number(v.altMsl(), 'f', 2) << ','
         << QString::number(v.relAlt(), 'f', 2) << ','
         << QString::number(v.climb(), 'f', 2) << ','
         << QString::number(v.groundspeed(), 'f', 2) << ','
         << v.headingDeg() << ','
         << v.batteryRemaining() << ','
         << v.missionCurrent() << ','
         << v.missionReached() << '\n';
    ++samples_;
    emit sampleWritten(samples_);
}

// ============================================================================
// 回放器
// ============================================================================
FlightLogPlayer::FlightLogPlayer(VehicleState* vehicle, QObject* parent)
    : QObject(parent)
    , vehicle_(vehicle)
{
    timer_.setInterval(100);
    connect(&timer_, &QTimer::timeout, this, &FlightLogPlayer::onStep);
}

bool FlightLogPlayer::load(const QString& filePath)
{
    stop();
    rows_.clear();
    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;
    while (!f.atEnd()) {
        const QByteArray line = f.readLine().trimmed();
        if (line.isEmpty())
            continue;
        const QByteArray first = line.left(4);
        if (first == "t_ms")       // 表头
            continue;
        rows_.append(QString::fromLatin1(line));
    }
    row_ = -1;
    emit progressChanged(0, rows_.size());
    emit logMessage(QObject::tr("[日志] 加载回放文件: %1 (%2 行)")
                    .arg(QFileInfo(filePath).fileName()).arg(rows_.size()));
    return !rows_.isEmpty();
}

void FlightLogPlayer::play()
{
    if (rows_.isEmpty())
        return;
    if (row_ < 0)
        row_ = 0;
    playing_ = true;
    timer_.setInterval(static_cast<int>(100.0 / speed_));
    timer_.start();
    emit logMessage(QObject::tr("[日志] 回放开始 (x%1)").arg(speed_));
}

void FlightLogPlayer::pause()
{
    if (!playing_)
        return;
    playing_ = false;
    timer_.stop();
    emit logMessage(QObject::tr("[日志] 回放暂停"));
}

void FlightLogPlayer::stop()
{
    playing_ = false;
    timer_.stop();
    row_ = -1;
    emit progressChanged(0, rows_.size());
}

void FlightLogPlayer::setSpeed(double speed)
{
    speed_ = std::max(0.5, std::min(10.0, speed));
    if (playing_)
        timer_.setInterval(static_cast<int>(100.0 / speed_));
}

bool FlightLogPlayer::jumpTo(int row)
{
    if (row < 0 || row >= rows_.size())
        return false;
    row_ = row;
    applyRow(row_);
    emit progressChanged(row_ + 1, rows_.size());
    return true;
}

void FlightLogPlayer::onStep()
{
    if (row_ + 1 >= rows_.size()) {
        pause();
        emit finished();
        emit logMessage(QObject::tr("[日志] 回放完成"));
        return;
    }
    ++row_;
    applyRow(row_);
    emit progressChanged(row_ + 1, rows_.size());
}

void FlightLogPlayer::applyRow(int i)
{
    if (!vehicle_ || i < 0 || i >= rows_.size())
        return;
    const QStringList c = rows_.at(i).split(',');
    if (c.size() < 14)
        return;
    const bool armed = c.at(1).toInt() != 0;
    const uint32_t main = static_cast<uint32_t>(c.at(2).toUInt());
    const uint32_t sub = static_cast<uint32_t>(c.at(3).toUInt());
    const double lat = c.at(4).toDouble();
    const double lon = c.at(5).toDouble();
    const double altMsl = c.at(6).toDouble();
    const double relAlt = c.at(7).toDouble();
    const double climb = c.at(8).toDouble();
    const double groundspeed = c.at(9).toDouble();
    const int heading = c.at(10).toInt();
    const int battery = c.at(11).toInt();
    const int cur = c.at(12).toInt();
    const int reached = c.at(13).toInt();

    uint8_t baseMode = mav::MODE_FLAG_CUSTOM_MODE_ENABLED;
    if (armed)
        baseMode |= mav::MODE_FLAG_SAFETY_ARMED;
    if (main == mav::PX4_MODE_AUTO)
        baseMode |= mav::MODE_FLAG_AUTO_ENABLED;
    else
        baseMode |= mav::MODE_FLAG_MANUAL_INPUT_ENABLED;
    vehicle_->updateHeartbeat(1, 1, mav::TYPE_QUADROTOR, mav::AUTOPILOT_PX4,
                              baseMode, mav::px4CustomModeEncode(main, sub),
                              mav::STATE_ACTIVE);
    vehicle_->updateGlobalPosition(lat, lon, altMsl, relAlt, 0, 0, 0, heading);
    vehicle_->updateVfrHud(groundspeed, groundspeed, climb, heading, 0);
    vehicle_->updateBattery(0, 0, battery, 0);
    vehicle_->updateMissionCurrent(cur, cur != 255);
    vehicle_->updateMissionReached(reached);
}

} // namespace skygcs
