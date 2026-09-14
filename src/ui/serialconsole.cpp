#include "serialconsole.h"

#include <QComboBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

namespace skygcs {

namespace {
// "01 02 0A FF" → 原始字节
QByteArray hexToBytes(const QString& hex)
{
    QByteArray out;
    const QString clean = hex.simplified().remove(' ');
    if (clean.size() % 2 != 0)
        return out;
    for (int i = 0; i < clean.size(); i += 2) {
        bool ok = false;
        out.append(static_cast<char>(clean.mid(i, 2).toInt(&ok, 16)));
        if (!ok)
            return QByteArray();
    }
    return out;
}
} // namespace

SerialConsole::SerialConsole(QWidget* parent)
    : QWidget(parent)
{
    frameProto_ = new SerialFrameProtocol(this);
    slcan_ = new SlcanProtocol(this);
    connect(frameProto_, &SerialFrameProtocol::logReceived, this, &SerialConsole::onLog);
    connect(slcan_, &SlcanProtocol::logReceived, this, &SerialConsole::onLog);
    connect(frameProto_, &SerialFrameProtocol::hdlcFrameReceived, this,
            [this](uint8_t addr, uint8_t func, const QByteArray& payload) {
                onLog(tr("[HDLC帧] addr=0x%1 func=0x%2 payload=%3")
                          .arg(addr, 2, 16, QChar('0'))
                          .arg(func, 2, 16, QChar('0'))
                          .arg(QString::fromLatin1(payload.toHex(' '))));
            });
    connect(frameProto_, &SerialFrameProtocol::modbusFrameReceived, this,
            [this](uint8_t addr, uint8_t func, const QByteArray& data) {
                onLog(tr("[Modbus] addr=%1 func=0x%2 data=%3")
                          .arg(addr)
                          .arg(func, 2, 16, QChar('0'))
                          .arg(QString::fromLatin1(data.toHex(' '))));
            });
    connect(slcan_, &SlcanProtocol::frameReceived, this,
            [this](const CanFrame& f) { onLog(tr("[CAN帧] %1").arg(f.toString())); });

    // ---- 发送区 ----
    comboProtocol_ = new QComboBox(this);
    comboProtocol_->addItem(tr("HDLC 帧协议 (RS485/422)"));
    comboProtocol_->addItem(tr("Modbus RTU (RS485)"));
    comboProtocol_->addItem(tr("SLCAN (CAN 桥)"));

    editAddr_ = new QLineEdit(QStringLiteral("01"), this);
    editFunc_ = new QLineEdit(QStringLiteral("10"), this);
    editData_ = new QLineEdit(QStringLiteral("01 02 03 04"), this);

    auto* sendBox = new QGroupBox(tr("发送 (RS485/RS422/CAN 载荷帧)"), this);
    auto* form = new QFormLayout(sendBox);
    form->addRow(tr("协议:"), comboProtocol_);
    form->addRow(tr("地址 (hex):"), editAddr_);
    form->addRow(tr("功能码 (hex):"), editFunc_);
    form->addRow(tr("数据 (hex):"), editData_);

    auto* btnRow = new QHBoxLayout;
    auto* btnSend = new QPushButton(tr("发送载荷帧"), this);
    btnSend->setStyleSheet("QPushButton{background:#2980b9;color:white;font-weight:600;}");
    auto* btnSendModbus = new QPushButton(tr("发送 Modbus"), this);
    btnOpenCan_ = new QPushButton(tr("打开 CAN"), this);
    btnCloseCan_ = new QPushButton(tr("关闭 CAN"), this);
    btnRow->addWidget(btnSend);
    btnRow->addWidget(btnSendModbus);
    btnRow->addWidget(btnOpenCan_);
    btnRow->addWidget(btnCloseCan_);
    form->addRow(btnRow);

    // CAN 参数
    auto* canBox = new QGroupBox(tr("CAN 帧 (SLCAN)"), this);
    auto* canForm = new QFormLayout(canBox);
    editCanId_ = new QLineEdit(QStringLiteral("123"), this);
    spinCanDlc_ = new QSpinBox(this);
    spinCanDlc_->setRange(0, 8);
    spinCanDlc_->setValue(2);
    editCanData_ = new QLineEdit(QStringLiteral("DE AD"), this);
    canForm->addRow(tr("ID (hex):"), editCanId_);
    canForm->addRow(tr("DLC:"), spinCanDlc_);
    canForm->addRow(tr("数据 (hex):"), editCanData_);
    auto* btnSendCan = new QPushButton(tr("发送 CAN 帧"), this);
    canForm->addRow(btnSendCan);

    logView_ = new QPlainTextEdit(this);
    logView_->setReadOnly(true);
    logView_->setMaximumBlockCount(500);
    logView_->setStyleSheet("QPlainTextEdit{background:#14181f;color:#9fe8a8;"
                            "font-family:Consolas,monospace;font-size:12px;}");

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(sendBox);
    layout->addWidget(canBox);
    layout->addWidget(new QLabel(tr("链路日志 (帧解析结果)"), this));
    layout->addWidget(logView_, 1);

    connect(btnSend, &QPushButton::clicked, this, &SerialConsole::onSendFrame);
    connect(btnSendModbus, &QPushButton::clicked, this, &SerialConsole::onSendModbus);
    connect(btnSendCan, &QPushButton::clicked, this, &SerialConsole::onSendCan);
    connect(btnOpenCan_, &QPushButton::clicked, this, &SerialConsole::onOpenCan);
    connect(btnCloseCan_, &QPushButton::clicked, this, &SerialConsole::onCloseCan);
    connect(this, &SerialConsole::logReceived, this, &SerialConsole::onLog);
}

void SerialConsole::attachLink(LinkInterface* link)
{
    link_ = link;
    frameProto_->attach(link);
    slcan_->attach(link);
}

void SerialConsole::detachLink()
{
    link_ = nullptr;
    frameProto_->detach();
    slcan_->detach();
}

bool parseHexByte(const QString& s, uint8_t& out)
{
    bool ok = false;
    const int v = s.toInt(&ok, 16);
    if (ok && v >= 0 && v <= 0xFF) {
        out = static_cast<uint8_t>(v);
        return true;
    }
    return false;
}

void SerialConsole::onSendFrame()
{
    uint8_t addr = 0, func = 0;
    if (!parseHexByte(editAddr_->text(), addr) || !parseHexByte(editFunc_->text(), func)) {
        onLog(tr("[错误] 地址/功能码必须是 1-2 位 hex"));
        return;
    }
    const QByteArray data = hexToBytes(editData_->text());
    if (data.isEmpty()) {
        onLog(tr("[错误] 数据必须是偶数个 hex 字符"));
        return;
    }
    if (!frameProto_->sendHdlcFrame(addr, func, data))
        onLog(tr("[错误] 链路未打开"));
}

void SerialConsole::onSendModbus()
{
    uint8_t addr = 0, func = 0;
    if (!parseHexByte(editAddr_->text(), addr) || !parseHexByte(editFunc_->text(), func)) {
        onLog(tr("[错误] 地址/功能码必须是 1-2 位 hex"));
        return;
    }
    const QByteArray data = hexToBytes(editData_->text());
    if (data.isEmpty()) {
        onLog(tr("[错误] 数据必须是偶数个 hex 字符"));
        return;
    }
    if (!frameProto_->sendModbus(addr, func, data))
        onLog(tr("[错误] 链路未打开"));
}

void SerialConsole::onSendCan()
{
    bool okId = false;
    const uint32_t id = editCanId_->text().toUInt(&okId, 16);
    if (!okId || id > 0x1FFFFFFF) {
        onLog(tr("[错误] CAN ID 非法"));
        return;
    }
    const QByteArray data = hexToBytes(editCanData_->text());
    if (data.size() != spinCanDlc_->value()) {
        onLog(tr("[错误] 数据长度与 DLC 不一致"));
        return;
    }
    CanFrame f{};
    f.id = id;
    f.extended = id > 0x7FF;
    f.dlc = static_cast<uint8_t>(spinCanDlc_->value());
    std::memcpy(f.data, data.constData(), static_cast<size_t>(data.size()));
    if (!slcan_->sendFrame(f))
        onLog(tr("[错误] 链路未打开"));
}

void SerialConsole::onOpenCan()
{
    if (!slcan_->sendOpenCan())
        onLog(tr("[错误] 链路未打开"));
}

void SerialConsole::onCloseCan()
{
    if (!slcan_->sendCloseCan())
        onLog(tr("[错误] 链路未打开"));
}

void SerialConsole::onLog(const QString& text)
{
    logView_->appendPlainText(text);
}

} // namespace skygcs
