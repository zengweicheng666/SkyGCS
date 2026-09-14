#include "parameterpanel.h"

#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>

#include "../comm/mavlinkendpoint.h"
#include "../core/vehicle.h"
#include "../mavlink/mavlink_names.h"

namespace skygcs {

ParameterPanel::ParameterPanel(MavlinkEndpoint* endpoint, QWidget* parent)
    : QWidget(parent)
    , endpoint_(endpoint)
    , vehicle_(endpoint ? endpoint->vehicle() : nullptr)
{
    auto* layout = new QVBoxLayout(this);

    auto* bar = new QHBoxLayout;
    btnRefresh_ = new QPushButton(tr("读取参数列表"), this);
    btnRead_ = new QPushButton(tr("读取选中"), this);
    status_ = new QLabel(tr("参数: 0 个 (点击\"读取参数列表\")"), this);
    bar->addWidget(btnRefresh_);
    bar->addWidget(btnRead_);
    bar->addStretch();
    bar->addWidget(status_);
    layout->addLayout(bar);

    table_ = new QTableWidget(0, 3, this);
    table_->setHorizontalHeaderLabels({ tr("参数名"), tr("值"), tr("类型") });
    table_->horizontalHeader()->setStretchLastSection(true);
    table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    table_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    table_->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
    table_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    layout->addWidget(table_);

    auto* hint = new QLabel(tr("提示: 双击「值」列修改参数, 回车后发送 PARAM_SET, 飞控回传确认后状态变绿"), this);
    hint->setStyleSheet("color:#7f8c8d;");
    layout->addWidget(hint);

    connect(btnRefresh_, &QPushButton::clicked, this, &ParameterPanel::refreshList);
    connect(btnRead_, &QPushButton::clicked, this, &ParameterPanel::readSelected);
    connect(table_, &QTableWidget::cellChanged, this, &ParameterPanel::onCellChanged);

    if (vehicle_) {
        connect(vehicle_, &VehicleState::paramChanged,
                this, &ParameterPanel::onParamChanged);
    }
}

void ParameterPanel::refreshList()
{
    if (endpoint_)
        endpoint_->requestParamList();
}

void ParameterPanel::readSelected()
{
    const int row = table_->currentRow();
    if (row < 0 || !endpoint_)
        return;
    const QTableWidgetItem* item = table_->item(row, 0);
    if (!item)
        return;
    endpoint_->readParam(item->text());
}

void ParameterPanel::onParamChanged(const QString& name, float value)
{
    if (!vehicle_)
        return;
    // 找到行并回填 (避免触发 cellChanged → 二次发送); 找不到则新增行
    selfEdit_ = true;
    int found = -1;
    for (int r = 0; r < table_->rowCount(); ++r) {
        QTableWidgetItem* n = table_->item(r, 0);
        if (n && n->text() == name) {
            found = r;
            break;
        }
    }
    if (found < 0) {
        const int r = table_->rowCount();
        table_->insertRow(r);
        auto* n = new QTableWidgetItem(name);
        n->setFlags(n->flags() & ~Qt::ItemIsEditable);
        table_->setItem(r, 0, n);
        table_->setItem(r, 1, new QTableWidgetItem(QString::number(value, 'g', 8)));
        auto* t = new QTableWidgetItem(mav::paramTypeName(vehicle_->paramType(name)));
        t->setFlags(t->flags() & ~Qt::ItemIsEditable);
        table_->setItem(r, 2, t);
        found = r;
    }
    QTableWidgetItem* v = table_->item(found, 1);
    if (v) {
        v->setText(QString::number(value, 'g', 8));
        v->setForeground(QColor(46, 204, 113));   // 绿色 = 已确认
    }
    QTableWidgetItem* t = table_->item(found, 2);
    if (t)
        t->setText(mav::paramTypeName(vehicle_->paramType(name)));
    selfEdit_ = false;
    status_->setText(tr("参数: %1 个 | 最近更新 %2 = %3")
                     .arg(vehicle_->paramCount()).arg(name).arg(value));
}

void ParameterPanel::onCellChanged(int row, int column)
{
    if (selfEdit_ || column != 1 || !endpoint_ || !vehicle_)
        return;
    const QTableWidgetItem* nameItem = table_->item(row, 0);
    QTableWidgetItem* valItem = table_->item(row, 1);
    if (!nameItem || !valItem)
        return;
    const QString name = nameItem->text();
    if (name.isEmpty())
        return;
    bool ok = false;
    const float value = valItem->text().toFloat(&ok);
    if (!ok) {
        status_->setText(tr("无效数值: %1").arg(valItem->text()));
        return;
    }
    // 标记待确认
    valItem->setForeground(QColor(230, 126, 34));
    endpoint_->setParam(name, value);
    vehicle_->markParamDirty(name, true);
}

} // namespace skygcs
