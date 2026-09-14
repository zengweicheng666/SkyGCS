#pragma once
// ============================================================================
// SkyGCS  指令面板: 解锁/上锁、起飞、降落、返航、模式切换 + ACK 反馈
// ============================================================================
#include <QWidget>

#include "../comm/mavlinkendpoint.h"

class QPushButton;
class QComboBox;
class QSpinBox;
class QPlainTextEdit;

namespace skygcs {

class CommandPanel : public QWidget {
    Q_OBJECT
public:
    explicit CommandPanel(MavlinkEndpoint* endpoint, QWidget* parent = nullptr);

private slots:
    void onArm();
    void onDisarm();
    void onTakeoff();
    void onLand();
    void onRtl();
    void onModeChanged(int index);
    void onAck(uint16_t command, uint8_t result, int progress, int resultParam2);
    void onLog(const QString& text);

private:
    MavlinkEndpoint* endpoint_;

    QPushButton* btnArm_ = nullptr;
    QPushButton* btnDisarm_ = nullptr;
    QPushButton* btnTakeoff_ = nullptr;
    QPushButton* btnLand_ = nullptr;
    QPushButton* btnRtl_ = nullptr;
    QComboBox* comboMode_ = nullptr;
    QSpinBox* spinAlt_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;
};

} // namespace skygcs
