#pragma once
// ============================================================================
// SkyGCS  链路配置对话框: UDP (仿真/真机) + 串口 (数传/RS485/RS422)
// ============================================================================
#include <QDialog>

class QComboBox;
class QLineEdit;
class QSpinBox;
class QCheckBox;

namespace skygcs {

class LinkDialog : public QDialog {
    Q_OBJECT
public:
    explicit LinkDialog(QWidget* parent = nullptr);

    // ---- UDP ----
    QLineEdit* udpLocalPort() const { return udpLocalPort_; }
    QLineEdit* udpTargetHost() const { return udpTargetHost_; }
    QLineEdit* udpTargetPort() const { return udpTargetPort_; }

    // ---- 串口 ----
    QComboBox* serialPort() const { return serialPort_; }
    QSpinBox* serialBaud() const { return serialBaud_; }

private:
    QLineEdit* udpLocalPort_ = nullptr;
    QLineEdit* udpTargetHost_ = nullptr;
    QLineEdit* udpTargetPort_ = nullptr;
    QComboBox* serialPort_ = nullptr;
    QSpinBox* serialBaud_ = nullptr;
};

} // namespace skygcs
