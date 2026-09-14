#pragma once
// ============================================================================
// SkyGCS  遥测面板: 姿态指示 + 关键数值 + 简单仪表条
// ============================================================================
#include <QWidget>

#include "../core/vehicle.h"

class QLabel;
class QProgressBar;

namespace skygcs {

class AttitudeIndicator;

class TelemetryPanel : public QWidget {
    Q_OBJECT
public:
    explicit TelemetryPanel(VehicleState* vehicle, QWidget* parent = nullptr);

private slots:
    void refresh();

private:
    VehicleState* vehicle_;
    AttitudeIndicator* horizon_ = nullptr;

    QLabel* lMode_ = nullptr;
    QLabel* lStatus_ = nullptr;
    QLabel* lGps_ = nullptr;
    QLabel* lPos_ = nullptr;
    QLabel* lAlt_ = nullptr;
    QLabel* lSpeed_ = nullptr;
    QLabel* lHeading_ = nullptr;
    QLabel* lBattery_ = nullptr;
    QLabel* lLoad_ = nullptr;
    QLabel* lLink_ = nullptr;
    QProgressBar* barBattery_ = nullptr;
    QProgressBar* barAlt_ = nullptr;
    QProgressBar* barSpeed_ = nullptr;
};

} // namespace skygcs
