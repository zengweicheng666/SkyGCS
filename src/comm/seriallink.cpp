#include "seriallink.h"

#include <QSerialPortInfo>

namespace skygcs {

SerialLink::SerialLink(QObject* parent)
    : LinkInterface(parent)
{
    connect(&port_, &QSerialPort::readyRead, this, [this]() {
        emit bytesReceived(port_.readAll());
    });
    connect(&port_, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError e) {
        if (e != QSerialPort::NoError) {
            close();
            emit connectedChanged(false);
        }
    });
}

QStringList SerialLink::availablePorts()
{
    QStringList list;
    for (const QSerialPortInfo& info : QSerialPortInfo::availablePorts())
        list << info.portName();
    return list;
}

QString SerialLink::detail() const
{
    return QStringLiteral("%1 @ %2 %3%4%5").arg(cfg_.portName)
        .arg(cfg_.baudRate)
        .arg(cfg_.dataBits == QSerialPort::Data8 ? "8" : "7")
        .arg(cfg_.parity == QSerialPort::NoParity ? "N" : (cfg_.parity == QSerialPort::EvenParity ? "E" : "O"))
        .arg(cfg_.stopBits == QSerialPort::OneStop ? "1" : "2");
}

bool SerialLink::open()
{
    if (isOpen())
        return true;
    port_.setPortName(cfg_.portName);
    port_.setBaudRate(cfg_.baudRate);
    port_.setDataBits(cfg_.dataBits);
    port_.setParity(cfg_.parity);
    port_.setStopBits(cfg_.stopBits);
    port_.setFlowControl(cfg_.flowControl);
    if (!port_.open(QIODevice::ReadWrite)) {
        emit connectedChanged(false);
        return false;
    }
    emit connectedChanged(true);
    return true;
}

void SerialLink::close()
{
    if (port_.isOpen())
        port_.close();
    emit connectedChanged(false);
}

bool SerialLink::isOpen() const
{
    return port_.isOpen();
}

void SerialLink::sendBytes(const QByteArray& data)
{
    if (port_.isOpen())
        port_.write(data);
}

} // namespace skygcs
