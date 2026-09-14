#pragma once
// ============================================================================
// SkyGCS  参数管理面板 (PARAM)
//   - 请求参数列表 (PARAM_REQUEST_LIST)
//   - 表格展示: 参数名 / 值 / 类型 / 状态
//   - 双击修改 → PARAM_SET, 飞控回传 PARAM_VALUE 确认
// ============================================================================
#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class QTableWidgetItem;

namespace skygcs {

class MavlinkEndpoint;
class VehicleState;

class ParameterPanel : public QWidget {
    Q_OBJECT
public:
    explicit ParameterPanel(MavlinkEndpoint* endpoint, QWidget* parent = nullptr);

public slots:
    void refreshList();
    void readSelected();
    void onParamChanged(const QString& name, float value);

private slots:
    void onCellChanged(int row, int column);

private:
    void populate();

    MavlinkEndpoint* endpoint_ = nullptr;
    VehicleState* vehicle_ = nullptr;

    QTableWidget* table_ = nullptr;
    QLabel* status_ = nullptr;
    QPushButton* btnRefresh_ = nullptr;
    QPushButton* btnRead_ = nullptr;
    bool selfEdit_ = false;      // 阻止回填触发二次发送
};

} // namespace skygcs
