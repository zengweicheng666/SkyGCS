#include "missionpanel.h"

#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>

#include "../comm/mavlinkendpoint.h"
#include "../core/vehicle.h"
#include "../mavlink/mavlink_names.h"

namespace skygcs {

namespace {
constexpr double kMetersPerDegLat = 111320.0;
} // namespace

MissionPanel::MissionPanel(MavlinkEndpoint* endpoint, QWidget* parent)
    : QWidget(parent)
    , endpoint_(endpoint)
    , vehicle_(endpoint->vehicle())
{
    auto* layout = new QVBoxLayout(this);

    auto* hint = new QLabel(tr("任务规划：编辑航点后「上传任务」，解锁后「开始任务」。"
                               "仿真器将依次飞越航点并在最后返航。"), this);
    hint->setWordWrap(true);
    hint->setStyleSheet("color:#8fa3bf;");
    layout->addWidget(hint);

    table_ = new QTableWidget(0, 5, this);
    table_->setHorizontalHeaderLabels({tr("序号"), tr("命令"), tr("纬度(°)"),
                                       tr("经度(°)"), tr("高度(m)")});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setEditTriggers(QAbstractItemView::DoubleClicked | QAbstractItemView::EditKeyPressed);
    layout->addWidget(table_, 1);

    auto* btns = new QHBoxLayout;
    auto* btnAdd = new QPushButton(tr("+ 添加航点"), this);
    auto* btnDel = new QPushButton(tr("删除选中"), this);
    auto* btnClr = new QPushButton(tr("清空"), this);
    btnUpload_ = new QPushButton(tr("⬆ 上传任务"), this);
    btnStart_ = new QPushButton(tr("▶ 开始任务"), this);
    for (auto* b : {btnAdd, btnDel, btnClr, btnUpload_, btnStart_})
        btns->addWidget(b);
    btns->addStretch();
    layout->addLayout(btns);

    status_ = new QLabel(tr("状态: 未上传任务"), this);
    status_->setStyleSheet("color:#7ee0b0;font-weight:600;");
    layout->addWidget(status_);

    connect(btnAdd, &QPushButton::clicked, this, &MissionPanel::addDefaultWaypoint);
    connect(btnDel, &QPushButton::clicked, this, &MissionPanel::removeSelected);
    connect(btnClr, &QPushButton::clicked, this, &MissionPanel::clearAll);
    connect(btnUpload_, &QPushButton::clicked, this, &MissionPanel::uploadMission);
    connect(btnStart_, &QPushButton::clicked, this, &MissionPanel::startMission);
    connect(vehicle_, &VehicleState::missionChanged, this, &MissionPanel::refreshStatus);
    connect(vehicle_, &VehicleState::stateChanged, this, &MissionPanel::refreshStatus);

    // 预置 3 个示例航点 (以家点为中心, 便于直接演示)
    addDefaultWaypoint();
    addDefaultWaypoint();
    addDefaultWaypoint();
}

// ---------------------------------------------------------------------------
void MissionPanel::addDefaultWaypoint()
{
    double lat = vehicle_->hasHome() ? vehicle_->homeLat() : 24.51;
    double lon = vehicle_->hasHome() ? vehicle_->homeLon() : 117.65;
    const int idx = table_->rowCount();
    // 方形绕飞: 北 60m → 北60m/东60m → 北120m/东60m
    const double dn[3] = {60.0, 60.0, 120.0};
    const double de[3] = {0.0, 60.0, 60.0};
    const double latM = dn[idx % 3] / kMetersPerDegLat;
    const double lonM = de[idx % 3] / (kMetersPerDegLat * std::cos(lat * M_PI / 180.0));
    addWaypointAt(lat + latM, lon + lonM, 8.0f);
}

void MissionPanel::addWaypointAt(double latDeg, double lonDeg, float altM)
{
    const int row = table_->rowCount();
    table_->insertRow(row);
    auto* item = new QTableWidgetItem(QString::number(row));
    item->setFlags(item->flags() & ~Qt::ItemIsEditable);
    table_->setItem(row, 0, item);
    table_->setItem(row, 1, new QTableWidgetItem(tr("NAV_WAYPOINT")));
    table_->setItem(row, 2, new QTableWidgetItem(QString::number(latDeg, 'f', 7)));
    table_->setItem(row, 3, new QTableWidgetItem(QString::number(lonDeg, 'f', 7)));
    table_->setItem(row, 4, new QTableWidgetItem(QString::number(altM, 'f', 1)));
    refreshStatus();
}

void MissionPanel::removeSelected()
{
    const int row = table_->currentRow();
    if (row < 0)
        return;
    table_->removeRow(row);
    renumber();
    refreshStatus();
}

void MissionPanel::clearAll()
{
    table_->setRowCount(0);
    endpoint_->abortMissionUpload();
    refreshStatus();
}

void MissionPanel::renumber()
{
    for (int i = 0; i < table_->rowCount(); ++i) {
        if (auto* it = table_->item(i, 0))
            it->setText(QString::number(i));
    }
}

// ---------------------------------------------------------------------------
void MissionPanel::uploadMission()
{
    QVector<MavlinkEndpoint::MissionItem> items;
    const int rows = table_->rowCount();
    for (int i = 0; i < rows; ++i) {
        auto* latIt = table_->item(i, 2);
        auto* lonIt = table_->item(i, 3);
        auto* altIt = table_->item(i, 4);
        if (!latIt || !lonIt || !altIt)
            continue;
        MavlinkEndpoint::MissionItem it;
        it.frame = mav::FRAME_GLOBAL_RELATIVE_ALT_INT;
        it.x = static_cast<int32_t>(latIt->text().toDouble() * 1e7);
        it.y = static_cast<int32_t>(lonIt->text().toDouble() * 1e7);
        it.z = altIt->text().toFloat();
        items.append(it);
    }
    if (items.isEmpty()) {
        status_->setText(tr("状态: 无航点可上传"));
        return;
    }
    if (!endpoint_->uploadMission(items))
        status_->setText(tr("状态: 上传失败 (无链路)"));
}

void MissionPanel::startMission()
{
    if (!endpoint_->startMission())
        status_->setText(tr("状态: 开始失败 (无链路)"));
}

void MissionPanel::refreshStatus()
{
    QStringList parts;
    parts << tr("已上传: %1 航点").arg(table_->rowCount());
    const int cur = vehicle_->missionCurrent();
    const int reached = vehicle_->missionReached();
    if (vehicle_->missionUploaded())
        parts << tr("飞控确认 ✓");
    if (cur != 255 && cur >= 0)
        parts << tr("当前航点 %1").arg(cur + 1);
    if (reached >= 0)
        parts << tr("已到达 %1").arg(reached + 1);
    if (vehicle_->missionActive())
        parts << tr("任务执行中");
    const int result = vehicle_->missionResult();
    if (result >= 0 && result != mav::MISSION_ACCEPTED)
        parts << tr("上传结果: %1").arg(mav::missionResultName(static_cast<uint8_t>(result)));
    status_->setText(tr("状态: ") + parts.join(" · "));
}

} // namespace skygcs
