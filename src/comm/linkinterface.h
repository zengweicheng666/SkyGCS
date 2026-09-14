#pragma once
// ============================================================================
// SkyGCS  链路抽象接口 (借鉴 QGC LinkInterface 设计)
// 派生: SerialLink(串口/RS485/RS422), UdpLink(UDP, 对接 PX4 SITL)
// ============================================================================
#include <QObject>
#include <QByteArray>
#include <QString>

namespace skygcs {

class LinkInterface : public QObject {
    Q_OBJECT
public:
    explicit LinkInterface(QObject* parent = nullptr) : QObject(parent) {}

    virtual bool open() = 0;
    virtual void close() = 0;
    virtual bool isOpen() const = 0;
    virtual QString name() const = 0;
    virtual QString detail() const = 0;          // 当前配置描述 (供状态栏)
    virtual void sendBytes(const QByteArray& data) = 0;

signals:
    void bytesReceived(const QByteArray& data);
    void connectedChanged(bool connected);
};

} // namespace skygcs
