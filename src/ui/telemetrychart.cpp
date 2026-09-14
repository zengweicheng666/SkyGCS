#include "telemetrychart.h"

#include <QTimer>
#include <QVBoxLayout>
#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

#include "../core/vehicle.h"

namespace skygcs {

TelemetryChart::TelemetryChart(VehicleState* vehicle, QWidget* parent)
    : QWidget(parent)
    , vehicle_(vehicle)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    auto* chart = new QChart;
    chart->setTitle(tr("遥测趋势 (高度/垂直速度/电量)"));
    chart->setBackgroundBrush(QColor("#10131a"));
    chart->setTitleBrush(QColor("#b8c4d6"));
    chart->legend()->setLabelColor(QColor("#b8c4d6"));
    chart->setAnimationOptions(QChart::NoAnimation);

    altSeries_ = new QLineSeries;
    altSeries_->setName(tr("高度 m"));
    altSeries_->setColor(QColor("#3498db"));
    climbSeries_ = new QLineSeries;
    climbSeries_->setName(tr("垂直速度 m/s"));
    climbSeries_->setColor(QColor("#e67e22"));
    battSeries_ = new QLineSeries;
    battSeries_->setName(tr("电量 %"));
    battSeries_->setColor(QColor("#2ecc71"));

    axisLeft_ = new QValueAxis;
    axisLeft_->setTitleText(tr("高度/速度"));
    axisLeft_->setRange(-5, 30);
    axisLeft_->setLabelFormat("%d");
    axisLeft_->setGridLineColor(QColor("#223"));
    axisLeft_->setTitleBrush(QColor("#8fa3bf"));
    axisLeft_->setLabelsColor(QColor("#8fa3bf"));
    axisRight_ = new QValueAxis;
    axisRight_->setTitleText(tr("电量 %"));
    axisRight_->setRange(0, 100);
    axisRight_->setLabelFormat("%d");
    axisRight_->setGridLineVisible(false);
    axisRight_->setTitleBrush(QColor("#8fa3bf"));
    axisRight_->setLabelsColor(QColor("#8fa3bf"));

    chart->addSeries(altSeries_);
    chart->addSeries(climbSeries_);
    chart->addSeries(battSeries_);
    chart->addAxis(axisLeft_, Qt::AlignLeft);
    chart->addAxis(axisRight_, Qt::AlignRight);
    altSeries_->attachAxis(axisLeft_);
    climbSeries_->attachAxis(axisLeft_);
    battSeries_->attachAxis(axisRight_);

    auto* view = new QChartView(chart, this);
    view->setRenderHint(QPainter::Antialiasing);
    layout->addWidget(view);

    timer_ = new QTimer(this);
    timer_->setInterval(500);
    connect(timer_, &QTimer::timeout, this, &TelemetryChart::onSample);
    timer_->start();
}

void TelemetryChart::onSample()
{
    t_ += 0.5;
    tBuf_.append(t_);
    altBuf_.append(vehicle_->relAlt());
    climbBuf_.append(vehicle_->climb());
    battBuf_.append(vehicle_->batteryRemaining() >= 0
                        ? vehicle_->batteryRemaining()
                        : 0.0);
    while (tBuf_.size() > maxSamples_) {
        tBuf_.removeFirst();
        altBuf_.removeFirst();
        climbBuf_.removeFirst();
        battBuf_.removeFirst();
    }
    // 窗口横轴跟随
    const double t0 = tBuf_.isEmpty() ? 0 : tBuf_.first();
    const double t1 = t0 + maxSamples_ * 0.5;
    axisLeft_->setRange(t0, t1);

    QVector<QPointF> ptsAlt, ptsClimb, ptsBatt;
    ptsAlt.reserve(tBuf_.size());
    ptsClimb.reserve(tBuf_.size());
    ptsBatt.reserve(tBuf_.size());
    for (int i = 0; i < tBuf_.size(); ++i) {
        ptsAlt.append(QPointF(tBuf_[i], altBuf_[i]));
        ptsClimb.append(QPointF(tBuf_[i], climbBuf_[i]));
        ptsBatt.append(QPointF(tBuf_[i], battBuf_[i]));
    }
    altSeries_->replace(ptsAlt);
    climbSeries_->replace(ptsClimb);
    battSeries_->replace(ptsBatt);
}

} // namespace skygcs
