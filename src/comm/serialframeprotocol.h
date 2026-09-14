#pragma once
// ============================================================================
// SkyGCS  载荷串口帧协议 (RS485/RS422/RS232 半双工总线)
//
// 支持两种常用帧格式 (自动识别):
//  1) HDLC 风格自定义帧:  F7 7E | addr | func | len | payload | CRC16-CCITT(2) | 7E
//     转义: 0x7E -> 0x7D 0x5E ; 0x7D -> 0x7D 0x5D
//     适用: 云台、传感器、扩展载荷等私有设备
//  2) Modbus RTU:         addr | func | data | CRC16-Modbus(2)
//     适用: RS485 工业总线 (变送器/舵机/电源管理)
// ============================================================================
#include <QObject>
#include <QByteArray>
#include <QTimer>

#include "linkinterface.h"

namespace skygcs {

class SerialFrameProtocol : public QObject {
    Q_OBJECT
public:
    explicit SerialFrameProtocol(QObject* parent = nullptr);

    void attach(LinkInterface* link);
    void detach();

    // ---- 发送 API ----
    bool sendHdlcFrame(uint8_t addr, uint8_t func, const QByteArray& payload);
    bool sendModbus(uint8_t addr, uint8_t func, const QByteArray& data);

    // ---- 校验 (公开, 便于测试) ----
    static quint16 crc16Ccitt(const QByteArray& data);
    static quint16 crc16Modbus(const QByteArray& data);

signals:
    void hdlcFrameReceived(uint8_t addr, uint8_t func, const QByteArray& payload);
    void modbusFrameReceived(uint8_t addr, uint8_t func, const QByteArray& data);
    void logReceived(const QString& text);

private slots:
    void onBytes(const QByteArray& data);
    void flushModbus();

private:
    void parseStream();

    LinkInterface* link_ = nullptr;
    QByteArray buf_;
    QTimer modbusTimer_;
};

} // namespace skygcs
