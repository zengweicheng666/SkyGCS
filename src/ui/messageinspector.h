#pragma once
// ============================================================================
// SkyGCS  MAVLink 消息检查器: 实时列出收到的全部消息及关键字段
// ============================================================================
#include <QWidget>

#include "../comm/mavlinkendpoint.h"

class QTableWidget;
class QCheckBox;
class QPlainTextEdit;

namespace skygcs {

class MessageInspector : public QWidget {
    Q_OBJECT
public:
    explicit MessageInspector(MavlinkEndpoint* endpoint, QWidget* parent = nullptr);

private slots:
    void onMessage(const skygcs::MavMessage& msg);
    void onCurrentRowChanged(int row);

private:
    void appendMessage(const skygcs::MavMessage& msg);

    MavlinkEndpoint* endpoint_;
    QTableWidget* table_ = nullptr;
    QPlainTextEdit* detail_ = nullptr;
    int count_ = 0;
};

} // namespace skygcs
