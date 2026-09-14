#include "telemetrypanel.h"

#include <QGridLayout>
#include <QLabel>
#include <QProgressBar>
#include <QVBoxLayout>

#include "attitudeindicator.h"

namespace skygcs {

namespace {
QLabel* makeNameLabel(const QString& text, QWidget* parent)
{
    auto* label = new QLabel(text, parent);
    label->setStyleSheet("color: #8a93a6;");
    return label;
}
QLabel* makeValueLabel(QWidget* parent)
{
    auto* label = new QLabel("-", parent);
    label->setStyleSheet("color: #e8e8e8; font-size: 15px; font-weight: 600;");
    return label;
}
} // namespace

TelemetryPanel::TelemetryPanel(VehicleState* vehicle, QWidget* parent)
    : QWidget(parent)
    , vehicle_(vehicle)
{
    horizon_ = new AttitudeIndicator(this);

    auto* left = new QVBoxLayout;
    left->addWidget(horizon_, 1);

    auto* grid = new QGridLayout;
    grid->setVerticalSpacing(8);
    grid->setHorizontalSpacing(12);

    int row = 0;
    auto addRow = [&](const QString& name, QLabel*& value) {
        grid->addWidget(makeNameLabel(name, this), row, 0);
        value = makeValueLabel(this);
        grid->addWidget(value, row, 1, 1, 2);
        ++row;
    };
    addRow(tr("飞行模式"), lMode_);
    addRow(tr("系统状态"), lStatus_);
    addRow(tr("GPS"), lGps_);
    addRow(tr("经纬度"), lPos_);
    addRow(tr("高度"), lAlt_);
    addRow(tr("速度"), lSpeed_);
    addRow(tr("航向"), lHeading_);
    addRow(tr("电池"), lBattery_);
    addRow(tr("CPU负载"), lLoad_);
    addRow(tr("链路"), lLink_);

    barBattery_ = new QProgressBar(this);
    barBattery_->setRange(0, 100);
    barBattery_->setTextVisible(true);
    grid->addWidget(makeNameLabel(tr("电量"), this), row, 0);
    grid->addWidget(barBattery_, row, 1, 1, 2);
    ++row;

    barAlt_ = new QProgressBar(this);
    barAlt_->setRange(0, 120);
    barAlt_->setTextVisible(true);
    grid->addWidget(makeNameLabel(tr("高度条"), this), row, 0);
    grid->addWidget(barAlt_, row, 1, 1, 2);
    ++row;

    barSpeed_ = new QProgressBar(this);
    barSpeed_->setRange(0, 30);
    barSpeed_->setTextVisible(true);
    grid->addWidget(makeNameLabel(tr("速度条"), this), row, 0);
    grid->addWidget(barSpeed_, row, 1, 1, 2);

    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(2, 1);

    auto* mainLayout = new QHBoxLayout(this);
    mainLayout->addLayout(left, 3);
    mainLayout->addLayout(grid, 4);

    connect(vehicle_, &VehicleState::stateChanged, this, &TelemetryPanel::refresh);
    refresh();
}

void TelemetryPanel::refresh()
{
    if (!vehicle_)
        return;

    horizon_->setAttitude(vehicle_->rollDeg(), vehicle_->pitchDeg(), vehicle_->armed());

    lMode_->setText(vehicle_->isOnline() ? vehicle_->modeName() : tr("离线"));
    lMode_->setStyleSheet(vehicle_->armed()
                              ? "color: #ff5c5c; font-size: 15px; font-weight: 700;"
                              : "color: #e8e8e8; font-size: 15px; font-weight: 600;");
    lStatus_->setText(vehicle_->systemStatusName());
    lGps_->setText(QStringLiteral("%1  %2 颗").arg(vehicle_->gpsFixName())
                       .arg(vehicle_->gpsSats()));
    lPos_->setText(QStringLiteral("%1, %2").arg(vehicle_->lat(), 0, 'f', 6)
                       .arg(vehicle_->lon(), 0, 'f', 6));
    lAlt_->setText(QStringLiteral("%1 m (MSL) / %2 m (REL)")
                       .arg(vehicle_->altMsl(), 0, 'f', 1)
                       .arg(vehicle_->relAlt(), 0, 'f', 1));
    lSpeed_->setText(QStringLiteral("空速 %1 | 地速 %2 | 爬升 %3 m/s")
                         .arg(vehicle_->airspeed(), 0, 'f', 1)
                         .arg(vehicle_->groundspeed(), 0, 'f', 1)
                         .arg(vehicle_->climb(), 0, 'f', 1));
    lHeading_->setText(QStringLiteral("%1°").arg(vehicle_->headingDeg()));
    lBattery_->setText(QStringLiteral("%1 V / %2 A / %3%")
                           .arg(vehicle_->batteryVoltage(), 0, 'f', 2)
                           .arg(vehicle_->batteryCurrent(), 0, 'f', 2)
                           .arg(vehicle_->batteryRemaining()));
    lLoad_->setText(QStringLiteral("%1%").arg(vehicle_->cpuLoad()));
    lLink_->setText(vehicle_->radioRemRssi() != 0
                        ? QStringLiteral("RSSI %1 / %2").arg(vehicle_->radioRssi())
                              .arg(vehicle_->radioRemRssi())
                        : tr("无数传信息"));

    const int batt = qBound(0, vehicle_->batteryRemaining(), 100);
    barBattery_->setValue(batt);
    barBattery_->setStyleSheet(batt > 25 ? "QProgressBar::chunk{background:#2ecc71;}"
                                         : "QProgressBar::chunk{background:#e74c3c;}");
    barAlt_->setValue(static_cast<int>(qBound(0.0, vehicle_->relAlt(), 120.0)));
    barAlt_->setFormat(tr("高度 %1 m").arg(vehicle_->relAlt(), 0, 'f', 0));
    barSpeed_->setValue(static_cast<int>(qBound(0.0, vehicle_->groundspeed(), 30.0)));
    barSpeed_->setFormat(tr("地速 %1 m/s").arg(vehicle_->groundspeed(), 0, 'f', 1));
}

} // namespace skygcs
