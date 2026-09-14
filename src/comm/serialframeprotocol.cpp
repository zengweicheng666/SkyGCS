#include "serialframeprotocol.h"

#include <QDateTime>
#include <QTimer>

namespace skygcs {

namespace {
constexpr quint8  FLAG = 0x7E;
constexpr quint8  ESC  = 0x7D;
constexpr quint8  ESC_XOR = 0x20;   // 转义后字节 = 原字节 ^ 0x20
constexpr size_t  MAX_FRAME = 300;
} // namespace

SerialFrameProtocol::SerialFrameProtocol(QObject* parent)
    : QObject(parent)
{
    modbusTimer_.setInterval(30);
    modbusTimer_.setSingleShot(true);
    connect(&modbusTimer_, &QTimer::timeout, this, &SerialFrameProtocol::flushModbus);
}

void SerialFrameProtocol::attach(LinkInterface* link)
{
    if (link_) {
        disconnect(link_, &LinkInterface::bytesReceived, this, &SerialFrameProtocol::onBytes);
    }
    link_ = link;
    if (link_)
        connect(link_, &LinkInterface::bytesReceived, this, &SerialFrameProtocol::onBytes);
    buf_.clear();
}

void SerialFrameProtocol::detach()
{
    attach(nullptr);
}

// ---------------------------------------------------------------------------
// CRC
// ---------------------------------------------------------------------------
quint16 SerialFrameProtocol::crc16Ccitt(const QByteArray& data)
{
    quint16 crc = 0xFFFF;
    for (char c : data) {
        crc ^= static_cast<quint16>(static_cast<quint8>(c)) << 8;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x8000)
                crc = static_cast<quint16>((crc << 1) ^ 0x1021);
            else
                crc = static_cast<quint16>(crc << 1);
        }
    }
    return crc;
}

quint16 SerialFrameProtocol::crc16Modbus(const QByteArray& data)
{
    quint16 crc = 0xFFFF;
    for (char c : data) {
        crc ^= static_cast<quint8>(c);
        for (int i = 0; i < 8; ++i) {
            if (crc & 1)
                crc = static_cast<quint16>((crc >> 1) ^ 0xA001);
            else
                crc = static_cast<quint16>(crc >> 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// 发送
// ---------------------------------------------------------------------------
bool SerialFrameProtocol::sendHdlcFrame(uint8_t addr, uint8_t func, const QByteArray& payload)
{
    if (!link_ || !link_->isOpen())
        return false;
    if (payload.size() > 250)
        return false;

    QByteArray body;
    body.append(static_cast<char>(addr));
    body.append(static_cast<char>(func));
    body.append(static_cast<char>(payload.size()));
    body.append(payload);
    const quint16 crc = crc16Ccitt(body);
    body.append(static_cast<char>(crc & 0xFF));
    body.append(static_cast<char>((crc >> 8) & 0xFF));

    // 帧封装 + 转义
    QByteArray frame;
    frame.append(static_cast<char>(FLAG));
    for (char c : body) {
        const quint8 b = static_cast<quint8>(c);
        if (b == FLAG || b == ESC) {
            frame.append(static_cast<char>(ESC));
            frame.append(static_cast<char>(b ^ ESC_XOR));
        } else {
            frame.append(static_cast<char>(b));
        }
    }
    frame.append(static_cast<char>(FLAG));

    link_->sendBytes(frame);
    emit logReceived(QObject::tr("[HDLC] TX addr=0x%1 func=0x%2 len=%3 crc=0x%4")
                         .arg(addr, 2, 16, QChar('0')).arg(func, 2, 16, QChar('0'))
                         .arg(payload.size()).arg(crc, 4, 16, QChar('0')));
    return true;
}

bool SerialFrameProtocol::sendModbus(uint8_t addr, uint8_t func, const QByteArray& data)
{
    if (!link_ || !link_->isOpen())
        return false;
    if (data.size() > 250)
        return false;

    QByteArray body;
    body.append(static_cast<char>(addr));
    body.append(static_cast<char>(func));
    body.append(data);
    const quint16 crc = crc16Modbus(body);
    body.append(static_cast<char>(crc & 0xFF));        // Modbus 低字节在前
    body.append(static_cast<char>((crc >> 8) & 0xFF));

    link_->sendBytes(body);
    emit logReceived(QObject::tr("[Modbus] TX addr=%1 func=0x%2 len=%3 crc=0x%4")
                         .arg(addr).arg(func, 2, 16, QChar('0'))
                         .arg(data.size()).arg(crc, 4, 16, QChar('0')));
    return true;
}

// ---------------------------------------------------------------------------
// 接收
// ---------------------------------------------------------------------------
void SerialFrameProtocol::onBytes(const QByteArray& data)
{
    buf_.append(data);
    parseStream();
    // 30ms 静默后解析 Modbus 帧 (3.5 字符间隔)
    modbusTimer_.start();
}

void SerialFrameProtocol::flushModbus()
{
    // 缓冲区开头若是 HDLC 帧残留, 先丢弃
    while (buf_.size() >= 1 && buf_.at(0) == static_cast<char>(FLAG))
        buf_.remove(0, 1);

    while (buf_.size() >= 4) {
        bool found = false;
        // 扫描可能帧长: addr func data(0..n) crc(2)
        for (int frameLen = 4; frameLen <= buf_.size(); ++frameLen) {
            if (frameLen > static_cast<int>(MAX_FRAME))
                break;
            const QByteArray body = buf_.left(frameLen - 2);
            const quint16 crcRx = static_cast<quint8>(buf_.at(frameLen - 2))
                                | (static_cast<quint8>(buf_.at(frameLen - 1)) << 8);
            if (crc16Modbus(body) == crcRx) {
                const uint8_t addr = static_cast<uint8_t>(body.at(0));
                const uint8_t func = static_cast<uint8_t>(body.at(1));
                const QByteArray data = body.mid(2);
                emit modbusFrameReceived(addr, func, data);
                emit logReceived(QObject::tr("[Modbus] RX addr=%1 func=0x%2 len=%3 crc OK")
                                     .arg(addr).arg(func, 2, 16, QChar('0')).arg(data.size()));
                buf_.remove(0, frameLen);
                found = true;
                break;
            }
        }
        if (!found)
            break;
    }
    // 缓冲上限保护
    if (buf_.size() > 4096)
        buf_.clear();
}

void SerialFrameProtocol::parseStream()
{
    // 1) 尝试按 HDLC 帧解析 (以 0x7E 为界)
    int flag = buf_.indexOf(static_cast<char>(FLAG));
    while (flag >= 0) {
        // 找下一个 0x7E 作为帧尾
        const int end = buf_.indexOf(static_cast<char>(FLAG), flag + 1);
        if (end < 0)
            break;
        QByteArray escaped = buf_.mid(flag + 1, end - flag - 1);

        // 去转义
        QByteArray body;
        bool ok = true;
        for (int i = 0; i < escaped.size(); ++i) {
            const quint8 b = static_cast<quint8>(escaped.at(i));
            if (b == ESC) {
                if (i + 1 >= escaped.size()) { ok = false; break; }
                body.append(static_cast<char>(static_cast<quint8>(escaped.at(i + 1)) ^ ESC_XOR));
                ++i;
            } else {
                body.append(static_cast<char>(b));
            }
        }

        if (ok && body.size() >= 5) {   // addr+func+len+crc(2)
            const uint8_t addr = static_cast<uint8_t>(body.at(0));
            const uint8_t func = static_cast<uint8_t>(body.at(1));
            const uint8_t len = static_cast<uint8_t>(body.at(2));
            if (3 + len + 2 == static_cast<size_t>(body.size())) {
                const QByteArray payload = body.mid(3, len);
                const quint16 crcRx = static_cast<quint8>(body.at(3 + len))
                                    | (static_cast<quint8>(body.at(4 + len)) << 8);
                const quint16 crcCalc = crc16Ccitt(body.left(3 + len));
                if (crcRx == crcCalc) {
                    emit hdlcFrameReceived(addr, func, payload);
                    emit logReceived(QObject::tr("[HDLC] RX addr=0x%1 func=0x%2 len=%3 crc OK")
                                         .arg(addr, 2, 16, QChar('0')).arg(func, 2, 16, QChar('0'))
                                         .arg(payload.size()));
                } else {
                    emit logReceived(QObject::tr("[HDLC] RX CRC 错误 (0x%1 != 0x%2)")
                                         .arg(crcRx, 4, 16, QChar('0'))
                                         .arg(crcCalc, 4, 16, QChar('0')));
                }
                buf_.remove(0, end + 1);
                flag = buf_.indexOf(static_cast<char>(FLAG));
                continue;
            }
        }
        // 非法帧: 移除到第一个 FLAG 为止
        buf_.remove(0, flag + 1);
        flag = buf_.indexOf(static_cast<char>(FLAG));
    }

    // 2) Modbus RTU 帧解析由 flushModbus() 在 30ms 静默后完成
    // 缓冲上限保护
    if (buf_.size() > 4096)
        buf_.clear();
}

} // namespace skygcs
