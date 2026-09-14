#pragma once
// ============================================================================
// SkyGCS  串口链路 (RS232 / RS485 / RS422)
// 注: RS485/RS422 为电气层标准, 软件层与 RS232 一致;
//     RS485 半双工方向控制由硬件(收发器)依据 RTS/DE 自动完成。
// ============================================================================
#include <QSerialPort>
#include <QSerialPortInfo>

#include "linkinterface.h"

namespace skygcs {

class SerialLink : public LinkInterface {
    Q_OBJECT
public:
    explicit SerialLink(QObject* parent = nullptr);

    struct Config {
        QString portName;
        int     baudRate = 57600;
        QSerialPort::DataBits dataBits = QSerialPort::Data8;
        QSerialPort::Parity   parity = QSerialPort::NoParity;
        QSerialPort::StopBits stopBits = QSerialPort::OneStop;
        QSerialPort::FlowControl flowControl = QSerialPort::NoFlowControl;
    };

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    // LinkInterface
    bool open() override;
    void close() override;
    bool isOpen() const override;
    QString name() const override { return QStringLiteral("串口 %1").arg(cfg_.portName); }
    QString detail() const override;
    void sendBytes(const QByteArray& data) override;

    // 便捷: 获取可用串口列表
    static QStringList availablePorts();

private:
    Config cfg_;
    QSerialPort port_;
};

} // namespace skygcs
