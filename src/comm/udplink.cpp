#include "udplink.h"

namespace skygcs {

UdpLink::UdpLink(QObject* parent)
    : LinkInterface(parent)
{
    connect(&socket_, &QUdpSocket::readyRead, this, [this]() {
        while (socket_.hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(static_cast<int>(socket_.pendingDatagramSize()));
            QHostAddress sender;
            quint16 senderPort = 0;
            socket_.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
            if (senderPort != 0) {
                lastPeer_ = sender;
                lastPeerPort_ = senderPort;
            }
            emit bytesReceived(datagram);
        }
    });
}

QString UdpLink::detail() const
{
    return QStringLiteral("本地:%1 → 对端:%2:%3")
        .arg(cfg_.localPort).arg(cfg_.remoteHost).arg(cfg_.remotePort);
}

bool UdpLink::open()
{
    if (isOpen())
        return true;
    if (!socket_.bind(QHostAddress::AnyIPv4, cfg_.localPort,
                      QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        emit connectedChanged(false);
        return false;
    }
    emit connectedChanged(true);
    return true;
}

void UdpLink::close()
{
    if (socket_.state() != QAbstractSocket::UnconnectedState)
        socket_.close();
    emit connectedChanged(false);
}

bool UdpLink::isOpen() const
{
    return socket_.state() != QAbstractSocket::UnconnectedState;
}

void UdpLink::sendBytes(const QByteArray& data)
{
    // 与 QGC 一致: 优先回复最近收到数据的对端, 否则发往配置目标
    if (lastPeerPort_ != 0)
        sendTo(data, lastPeer_, lastPeerPort_);
    else
        sendTo(data, QHostAddress(cfg_.remoteHost), cfg_.remotePort);
}

void UdpLink::sendTo(const QByteArray& data, const QHostAddress& addr, quint16 port)
{
    if (isOpen())
        socket_.writeDatagram(data, addr, port);
}

} // namespace skygcs
