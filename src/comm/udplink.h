#pragma once
// ============================================================================
// SkyGCS  UDP 链路 (对接 PX4 SITL / 无人机数传 UDP 模式)
// 行为与 QGC 一致: 绑定本地端口(默认 14550)接收遥测, 回复发往对端地址
// ============================================================================
#include <QUdpSocket>

#include "linkinterface.h"

namespace skygcs {

class UdpLink : public LinkInterface {
    Q_OBJECT
public:
    explicit UdpLink(QObject* parent = nullptr);

    struct Config {
        quint16 localPort = 14550;       // 地面站监听端口
        QString remoteHost = "127.0.0.1"; // 默认目标 (SITL)
        quint16 remotePort = 14550;
    };

    void setConfig(const Config& cfg) { cfg_ = cfg; }
    const Config& config() const { return cfg_; }

    // LinkInterface
    bool open() override;
    void close() override;
    bool isOpen() const override;
    QString name() const override { return QStringLiteral("UDP"); }
    QString detail() const override;
    void sendBytes(const QByteArray& data) override;

    // 回复到最近接收数据的对端 (与 QGC 相同策略)
    void sendTo(const QByteArray& data, const QHostAddress& addr, quint16 port);

private:
    Config cfg_;
    QUdpSocket socket_;
    QHostAddress lastPeer_;
    quint16 lastPeerPort_ = 0;
};

} // namespace skygcs
