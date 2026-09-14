#include "attitudeindicator.h"

#include <QPainter>
#include <QtMath>

namespace skygcs {

AttitudeIndicator::AttitudeIndicator(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(200, 200);
}

void AttitudeIndicator::setAttitude(double rollDeg, double pitchDeg, bool armed)
{
    roll_ = rollDeg;
    pitch_ = pitchDeg;
    armed_ = armed;
    update();
}

void AttitudeIndicator::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    const int side = qMin(width(), height());
    const QPointF center(width() / 2.0, height() / 2.0);
    const double R = side * 0.44;

    // 背景圆
    p.setBrush(QColor(30, 34, 44));
    p.setPen(QPen(QColor(90, 100, 120), 2));
    p.drawEllipse(center, R, R);

    // 地平仪盘: 按横滚旋转, 按俯仰平移
    p.save();
    p.translate(center);
    p.rotate(-roll_);
    const double pitchShift = pitch_ * (R / 55.0);   // 55deg 满量程
    p.translate(0, pitchShift);

    // 天空 (上) 与大地 (下)
    const double half = R * 2;
    QLinearGradient sky(0, -half, 0, 0);
    sky.setColorAt(0, QColor(70, 130, 220));
    sky.setColorAt(1, QColor(150, 200, 250));
    QLinearGradient ground(0, 0, 0, half);
    ground.setColorAt(0, QColor(150, 100, 40));
    ground.setColorAt(1, QColor(90, 55, 20));

    p.setPen(Qt::NoPen);
    p.setBrush(sky);
    p.drawRect(QRectF(-half, -half, half * 2, half));
    p.setBrush(ground);
    p.drawRect(QRectF(-half, 0, half * 2, half));

    // 地平线
    p.setPen(QPen(Qt::white, 2.5));
    p.drawLine(QPointF(-R * 1.25, 0), QPointF(R * 1.25, 0));

    // 俯仰刻度 (-30..+30, 每 10 度)
    p.setPen(QPen(Qt::white, 1.2));
    QFont f = p.font();
    f.setPixelSize(11);
    p.setFont(f);
    for (int deg = -30; deg <= 30; deg += 10) {
        if (deg == 0)
            continue;
        const double y = -deg * (R / 55.0);
        const double w = (qAbs(deg) == 30) ? R * 0.45 : R * 0.3;
        p.drawLine(QPointF(-w, y), QPointF(w, y));
        p.drawText(QRectF(-R * 0.9, y - 8, R * 0.4, 16), Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(deg));
        p.drawText(QRectF(R * 0.5, y - 8, R * 0.4, 16), Qt::AlignLeft | Qt::AlignVCenter,
                   QString::number(deg));
    }
    p.restore();

    // 固定指针 (机身)
    p.setPen(QPen(QColor(255, 120, 40), 3));
    p.setBrush(QColor(255, 120, 40));
    QPolygonF tri;
    tri << QPointF(center.x(), center.y() - R * 0.85)
        << QPointF(center.x() - 9, center.y() - R * 0.55)
        << QPointF(center.x() + 9, center.y() - R * 0.55);
    p.drawPolygon(tri);

    // 中心符号
    p.setPen(QPen(Qt::white, 2));
    p.drawLine(QPointF(center.x() - R * 0.18, center.y()), QPointF(center.x() - R * 0.06, center.y()));
    p.drawLine(QPointF(center.x() + R * 0.06, center.y()), QPointF(center.x() + R * 0.18, center.y()));
    p.drawLine(QPointF(center.x(), center.y() - R * 0.06), QPointF(center.x(), center.y() + R * 0.06));

    // 横滚刻度弧 (每 30 度)
    p.setPen(QPen(QColor(220, 220, 230), 1.5));
    for (int deg = -60; deg <= 60; deg += 30) {
        if (deg == 0)
            continue;
        const double a = qDegreesToRadians(static_cast<double>(deg) - 90.0);
        p.drawLine(QPointF(center.x() + R * 0.82 * qCos(a), center.y() + R * 0.82 * qSin(a)),
                   QPointF(center.x() + R * 0.92 * qCos(a), center.y() + R * 0.92 * qSin(a)));
    }

    // 外圈信息
    p.setPen(Qt::white);
    p.drawText(QRectF(center.x() - R, center.y() - R, R * 2, 20),
               Qt::AlignHCenter | Qt::AlignTop,
               QStringLiteral("ROLL %1°  PITCH %2°").arg(roll_, 0, 'f', 1).arg(pitch_, 0, 'f', 1));

    // 解锁指示
    if (armed_) {
        p.setPen(QPen(QColor(255, 90, 60), 2.5));
        p.drawText(QRectF(center.x() - R, center.y() + R - 24, R * 2, 20),
                   Qt::AlignHCenter, "ARMED");
    }
}

} // namespace skygcs
