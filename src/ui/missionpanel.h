#pragma once
// ============================================================================
// SkyGCS  任务规划面板 (Mission)
//   - 航点编辑 (添加/删除/清空)
//   - 上传任务 (MISSION_COUNT/ITEM_INT 异步握手)
//   - 开始任务 (CMD_MISSION_START) / 进度显示
// ============================================================================
#include <QWidget>

class QLabel;
class QPushButton;
class QTableWidget;
class QTableWidgetItem;

namespace skygcs {

class MavlinkEndpoint;
class VehicleState;

class MissionPanel : public QWidget {
    Q_OBJECT
public:
    explicit MissionPanel(MavlinkEndpoint* endpoint, QWidget* parent = nullptr);

    // 供脚本/测试: 以家点为基准添加示例航点
    void addWaypointAt(double latDeg, double lonDeg, float altM);

public slots:
    void addDefaultWaypoint();
    void removeSelected();
    void clearAll();
    void uploadMission();
    void startMission();
    void refreshStatus();

private:
    void renumber();
    MavlinkEndpoint* endpoint_ = nullptr;
    VehicleState* vehicle_ = nullptr;

    QTableWidget* table_ = nullptr;
    QLabel* status_ = nullptr;
    QPushButton* btnUpload_ = nullptr;
    QPushButton* btnStart_ = nullptr;
};

} // namespace skygcs
