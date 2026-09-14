#pragma once
// ============================================================================
// SkyGCS  载荷串口控制台 (RS485/RS422/RS232 + Modbus RTU + SLCAN/CAN)
// 演示串口链路分层: LinkInterface → 帧协议 (HDLC / Modbus / SLCAN)
// ============================================================================
#include <QWidget>

#include "../comm/linkinterface.h"
#include "../comm/serialframeprotocol.h"
#include "../comm/slcanprotocol.h"

class QComboBox;
class QSpinBox;
class QPushButton;
class QPlainTextEdit;
class QLineEdit;
class QCheckBox;

namespace skygcs {

class SerialConsole : public QWidget {
    Q_OBJECT
public:
    explicit SerialConsole(QWidget* parent = nullptr);

    // 由主窗口在打开串口后调用
    void attachLink(LinkInterface* link);
    void detachLink();

signals:
    void logReceived(const QString& text);

private slots:
    void onSendFrame();
    void onSendModbus();
    void onSendCan();
    void onOpenCan();
    void onCloseCan();
    void onLog(const QString& text);

private:
    LinkInterface* link_ = nullptr;
    SerialFrameProtocol* frameProto_ = nullptr;
    SlcanProtocol* slcan_ = nullptr;

    QComboBox* comboProtocol_ = nullptr;

    // HDLC / Modbus
    QLineEdit* editAddr_ = nullptr;
    QLineEdit* editFunc_ = nullptr;
    QLineEdit* editData_ = nullptr;

    // SLCAN
    QLineEdit* editCanId_ = nullptr;
    QSpinBox* spinCanDlc_ = nullptr;
    QLineEdit* editCanData_ = nullptr;
    QPushButton* btnOpenCan_ = nullptr;
    QPushButton* btnCloseCan_ = nullptr;

    QPlainTextEdit* logView_ = nullptr;
};

} // namespace skygcs
