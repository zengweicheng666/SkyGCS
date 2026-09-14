#pragma once
// ============================================================================
// SkyGCS  人工地平仪 (姿态指示器)
// 自定义绘制: 天空/大地渐变 + 地平线 + 俯仰刻度 + 横滚指针
// ============================================================================
#include <QWidget>

namespace skygcs {

class AttitudeIndicator : public QWidget {
    Q_OBJECT
public:
    explicit AttitudeIndicator(QWidget* parent = nullptr);

    void setAttitude(double rollDeg, double pitchDeg, bool armed = false);

    QSize sizeHint() const override { return QSize(280, 280); }

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    double roll_ = 0;
    double pitch_ = 0;
    bool armed_ = false;
};

} // namespace skygcs
