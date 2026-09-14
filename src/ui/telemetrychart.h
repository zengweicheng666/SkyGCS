#pragma once
// ============================================================================
// SkyGCS  遥测实时曲线 (QtCharts)
//   高度 / 垂直速度 / 电量 滚动窗口 (默认 60s, 500ms 采样)
// ============================================================================
#include <QWidget>

#include <QVector>

class QTimer;

QT_BEGIN_NAMESPACE
class QLineSeries;
class QValueAxis;
QT_END_NAMESPACE

namespace skygcs {

class VehicleState;

class TelemetryChart : public QWidget {
    Q_OBJECT
public:
    explicit TelemetryChart(VehicleState* vehicle, QWidget* parent = nullptr);

private slots:
    void onSample();

private:
    VehicleState* vehicle_ = nullptr;
    QTimer* timer_ = nullptr;

    QLineSeries* altSeries_ = nullptr;     // 高度 m (左轴)
    QLineSeries* climbSeries_ = nullptr;   // 垂直速度 m/s (左轴)
    QLineSeries* battSeries_ = nullptr;    // 电量 % (右轴)
    QValueAxis* axisLeft_ = nullptr;
    QValueAxis* axisRight_ = nullptr;

    QVector<double> tBuf_;
    QVector<double> altBuf_;
    QVector<double> climbBuf_;
    QVector<double> battBuf_;
    int maxSamples_ = 120;   // 500ms × 60s
    double t_ = 0;
};

} // namespace skygcs
