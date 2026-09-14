#include "flightlogpanel.h"

#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSlider>
#include <QTime>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>

#include "../comm/mavlinkendpoint.h"
#include "../core/flightlog.h"
#include "../core/vehicle.h"

namespace skygcs {

namespace {
QString defaultLogPath()
{
    const QString dir = QDir::current().filePath(QStringLiteral("logs"));
    QDir().mkpath(dir);
    const QString stamp = QTime::currentTime().toString("hhmmss");
    return dir + QStringLiteral("/flight_%1.csv").arg(stamp);
}
} // namespace

FlightLogPanel::FlightLogPanel(MavlinkEndpoint* endpoint, QWidget* parent)
    : QWidget(parent)
    , endpoint_(endpoint)
{
    VehicleState* vehicle = endpoint ? endpoint->vehicle() : nullptr;
    recorder_ = new FlightLogRecorder(vehicle, this);
    player_ = new FlightLogPlayer(vehicle, this);

    auto* layout = new QVBoxLayout(this);

    // ---- 记录区 ----
    auto* recTitle = new QLabel(tr("■ 遥测记录 (CSV, 500ms 采样)"), this);
    recTitle->setStyleSheet("font-weight:bold;");
    layout->addWidget(recTitle);

    auto* recRow = new QHBoxLayout;
    recPath_ = new QLineEdit(defaultLogPath(), this);
    recStatus_ = new QLabel(tr("未记录"), this);
    btnRecStart_ = new QPushButton(tr("开始记录"), this);
    btnRecStop_ = new QPushButton(tr("停止"), this);
    btnRecStop_->setEnabled(false);
    recRow->addWidget(recPath_, 1);
    recRow->addWidget(btnRecStart_);
    recRow->addWidget(btnRecStop_);
    layout->addLayout(recRow);
    layout->addWidget(recStatus_);

    // ---- 回放区 ----
    auto* playTitle = new QLabel(tr("▶ 回放"), this);
    playTitle->setStyleSheet("font-weight:bold; margin-top:12px;");
    layout->addWidget(playTitle);

    auto* playRow = new QHBoxLayout;
    playFile_ = new QLabel(tr("未加载文件"), this);
    btnOpen_ = new QPushButton(tr("打开…"), this);
    btnPlay_ = new QPushButton(tr("播放"), this);
    btnStop_ = new QPushButton(tr("停止"), this);
    btnPlay_->setEnabled(false);
    btnStop_->setEnabled(false);
    playRow->addWidget(playFile_, 1);
    playRow->addWidget(btnOpen_);
    playRow->addWidget(btnPlay_);
    playRow->addWidget(btnStop_);
    layout->addLayout(playRow);

    auto* speedRow = new QHBoxLayout;
    speedLabel_ = new QLabel(tr("速度 x1"), this);
    speedSlider_ = new QSlider(Qt::Horizontal, this);
    speedSlider_->setRange(1, 10);
    speedSlider_->setValue(1);
    speedRow->addWidget(new QLabel(tr("x0.5"), this));
    speedRow->addWidget(speedSlider_, 1);
    speedRow->addWidget(new QLabel(tr("x10"), this));
    speedRow->addWidget(speedLabel_);
    layout->addLayout(speedRow);

    progress_ = new QProgressBar(this);
    progress_->setRange(0, 100);
    layout->addWidget(progress_);
    layout->addStretch();

    // ---- 信号 ----
    connect(btnRecStart_, &QPushButton::clicked, this, &FlightLogPanel::startRecording);
    connect(btnRecStop_, &QPushButton::clicked, this, &FlightLogPanel::stopRecording);
    connect(btnOpen_, &QPushButton::clicked, this, &FlightLogPanel::openAndPlay);
    connect(btnPlay_, &QPushButton::clicked, this, &FlightLogPanel::playPause);
    connect(btnStop_, &QPushButton::clicked, this, &FlightLogPanel::stopPlay);
    connect(speedSlider_, &QSlider::valueChanged, this, [this](int v) {
        speedLabel_->setText(tr("速度 x%1").arg(v));
        if (player_)
            player_->setSpeed(static_cast<double>(v));
    });
    connect(recorder_, &FlightLogRecorder::logMessage, this, &FlightLogPanel::onLog);
    connect(player_, &FlightLogPlayer::logMessage, this, &FlightLogPanel::onLog);
    connect(player_, &FlightLogPlayer::progressChanged, this, &FlightLogPanel::onProgress);
    connect(player_, &FlightLogPlayer::finished, this, [this]() {
        btnPlay_->setText(tr("播放"));
        setPlayControls(true, false);
    });
}

void FlightLogPanel::startRecording()
{
    if (recorder_->recording())
        return;
    const QString path = recPath_->text().trimmed();
    if (recorder_->start(path)) {
        setRecControls(true);
        recStatus_->setText(tr("记录中 → %1").arg(path));
    } else {
        recStatus_->setText(tr("无法打开文件: %1").arg(path));
    }
}

void FlightLogPanel::stopRecording()
{
    if (!recorder_->recording())
        return;
    recorder_->stop();
    setRecControls(false);
    recStatus_->setText(tr("已保存 %1 个采样点").arg(recorder_->sampleCount()));
}

void FlightLogPanel::openAndPlay()
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("打开飞行日志"), QStringLiteral("logs"),
        tr("SkyGCS 飞行日志 (*.csv);;所有文件 (*)"));
    if (path.isEmpty())
        return;
    if (!player_->load(path)) {
        onLog(tr("[日志] 加载失败: %1").arg(path));
        return;
    }
    playFile_->setText(QFileInfo(path).fileName());
    setPlayControls(true, false);
    player_->play();
    setPlayControls(true, true);
}

void FlightLogPanel::playPause()
{
    if (!player_->loaded())
        return;
    if (player_->playing()) {
        player_->pause();
        btnPlay_->setText(tr("播放"));
    } else {
        player_->play();
        btnPlay_->setText(tr("暂停"));
    }
}

void FlightLogPanel::stopPlay()
{
    player_->stop();
    btnPlay_->setText(tr("播放"));
    setPlayControls(true, false);
    progress_->setValue(0);
}

void FlightLogPanel::onProgress(int row, int total)
{
    progress_->setMaximum(std::max(1, total));
    progress_->setValue(row);
}

void FlightLogPanel::onLog(const QString& text)
{
    if (endpoint_)
        emit endpoint_->logMessage(text);
}

void FlightLogPanel::setRecControls(bool recording)
{
    btnRecStart_->setEnabled(!recording);
    btnRecStop_->setEnabled(recording);
    recPath_->setEnabled(!recording);
}

void FlightLogPanel::setPlayControls(bool loaded, bool playing)
{
    btnPlay_->setEnabled(loaded);
    btnStop_->setEnabled(loaded);
    if (!loaded)
        btnPlay_->setText(tr("播放"));
    else if (playing)
        btnPlay_->setText(tr("暂停"));
    else
        btnPlay_->setText(tr("播放"));
}

} // namespace skygcs
