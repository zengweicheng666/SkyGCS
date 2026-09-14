#include "slcanprotocol.h"

#include <QStringList>

namespace skygcs {

namespace {
// SLCAN 标准帧头 't' + 3 位 ID + 1 位 DLC + 数据
bool parseStandard(const QByteArray& line, CanFrame& out)
{
    if (line.size() < 5 || line.at(0) != 't')
        return false;
    bool okId = false, okDlc = false;
    const quint32 id = line.mid(1, 3).toUInt(&okId, 16);
    const int dlc = line.mid(4, 1).toInt(&okDlc, 16);
    if (!okId || !okDlc || dlc < 0 || dlc > 8)
        return false;
    if (line.size() != 5 + dlc * 2)
        return false;
    out.id = id & 0x7FF;
    out.extended = false;
    out.rtr = false;
    out.dlc = static_cast<uint8_t>(dlc);
    for (int i = 0; i < dlc; ++i) {
        bool ok = false;
        out.data[i] = static_cast<uint8_t>(line.mid(5 + i * 2, 2).toUInt(&ok, 16));
        if (!ok)
            return false;
    }
    return true;
}

// SLCAN 扩展帧头 'T' + 8 位 ID + 1 位 DLC + 数据
bool parseExtended(const QByteArray& line, CanFrame& out)
{
    if (line.size() < 10 || line.at(0) != 'T')
        return false;
    bool okId = false, okDlc = false;
    const quint32 id = line.mid(1, 8).toUInt(&okId, 16);
    const int dlc = line.mid(9, 1).toInt(&okDlc, 16);
    if (!okId || !okDlc || dlc < 0 || dlc > 8)
        return false;
    if (line.size() != 10 + dlc * 2)
        return false;
    out.id = id & 0x1FFFFFFF;
    out.extended = true;
    out.rtr = false;
    out.dlc = static_cast<uint8_t>(dlc);
    for (int i = 0; i < dlc; ++i) {
        bool ok = false;
        out.data[i] = static_cast<uint8_t>(line.mid(10 + i * 2, 2).toUInt(&ok, 16));
        if (!ok)
            return false;
    }
    return true;
}
} // namespace

QString CanFrame::toString() const
{
    QString s = extended ? QStringLiteral("CAN-E %1").arg(id, 8, 16, QChar('0'))
                         : QStringLiteral("CAN-S %1").arg(id, 3, 16, QChar('0'));
    if (rtr) {
        s += QStringLiteral(" RTR dlc=%1").arg(dlc);
    } else {
        s += QStringLiteral(" dlc=%1").arg(dlc);
        for (int i = 0; i < dlc; ++i)
            s += QStringLiteral(" %1").arg(data[i], 2, 16, QChar('0'));
    }
    return s;
}

SlcanProtocol::SlcanProtocol(QObject* parent)
    : QObject(parent)
{
}

void SlcanProtocol::attach(LinkInterface* link)
{
    if (link_) {
        disconnect(link_, &LinkInterface::bytesReceived, this, &SlcanProtocol::onBytes);
    }
    link_ = link;
    if (link_)
        connect(link_, &LinkInterface::bytesReceived, this, &SlcanProtocol::onBytes);
    lineBuf_.clear();
}

void SlcanProtocol::detach()
{
    attach(nullptr);
}

bool SlcanProtocol::sendOpenCan()
{
    if (!link_ || !link_->isOpen())
        return false;
    link_->sendBytes("Z\r");
    emit logReceived(QObject::tr("[SLCAN] TX: Z (打开 CAN 通道)"));
    return true;
}

bool SlcanProtocol::sendCloseCan()
{
    if (!link_ || !link_->isOpen())
        return false;
    link_->sendBytes("z\r");
    emit logReceived(QObject::tr("[SLCAN] TX: z (关闭 CAN 通道)"));
    return true;
}

bool SlcanProtocol::sendVersion()
{
    if (!link_ || !link_->isOpen())
        return false;
    link_->sendBytes("V\r");
    emit logReceived(QObject::tr("[SLCAN] TX: V (查询版本)"));
    return true;
}

bool SlcanProtocol::sendFrame(const CanFrame& frame)
{
    if (!link_ || !link_->isOpen())
        return false;
    if (frame.dlc > 8)
        return false;

    QString cmd;
    if (frame.extended)
        cmd = QStringLiteral("T%1%2")
                  .arg(frame.id & 0x1FFFFFFF, 8, 16, QChar('0'))
                  .arg(frame.dlc, 1, 16);
    else
        cmd = QStringLiteral("t%1%2")
                  .arg(frame.id & 0x7FF, 3, 16, QChar('0'))
                  .arg(frame.dlc, 1, 16);
    if (!frame.rtr) {
        for (int i = 0; i < frame.dlc; ++i)
            cmd += QStringLiteral("%1").arg(frame.data[i], 2, 16, QChar('0'));
    }
    cmd += QStringLiteral("\r");

    link_->sendBytes(cmd.toLatin1());
    emit logReceived(QObject::tr("[SLCAN] TX: %1 (%2)").arg(cmd.trimmed(), frame.toString()));
    return true;
}

void SlcanProtocol::onBytes(const QByteArray& data)
{
    lineBuf_.append(data);
    int nl = lineBuf_.indexOf('\r');
    while (nl >= 0) {
        const QByteArray line = lineBuf_.left(nl);
        lineBuf_.remove(0, nl + 1);
        parseLine(line);
        nl = lineBuf_.indexOf('\r');
    }
    if (lineBuf_.size() > 512)
        lineBuf_.clear();
}

void SlcanProtocol::parseLine(const QByteArray& line)
{
    if (line.isEmpty())
        return;
    const char c = line.at(0);

    if (c == 't') {
        CanFrame f;
        if (parseStandard(line, f)) {
            emit frameReceived(f);
            emit logReceived(QObject::tr("[SLCAN] RX: %1").arg(f.toString()));
        }
        return;
    }
    if (c == 'T') {
        CanFrame f;
        if (parseExtended(line, f)) {
            emit frameReceived(f);
            emit logReceived(QObject::tr("[SLCAN] RX: %1").arg(f.toString()));
        }
        return;
    }
    // 其余 ASCII 响应原样记入日志
    emit logReceived(QObject::tr("[SLCAN] RX: %1").arg(QString::fromLatin1(line)));
}

} // namespace skygcs
