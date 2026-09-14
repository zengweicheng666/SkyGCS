#include "linkdialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QSerialPortInfo>
#include <QSpinBox>
#include <QVBoxLayout>

namespace skygcs {

LinkDialog::LinkDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("新建链路"));
    setMinimumWidth(420);

    auto* udpBox = new QGroupBox(tr("UDP (PX4 SITL / 真机遥测)"), this);
    auto* udpForm = new QFormLayout(udpBox);
    udpLocalPort_ = new QLineEdit(QStringLiteral("14550"), this);
    udpTargetHost_ = new QLineEdit(QStringLiteral("127.0.0.1"), this);
    udpTargetPort_ = new QLineEdit(QStringLiteral("14550"), this);
    udpForm->addRow(tr("本地监听端口:"), udpLocalPort_);
    udpForm->addRow(tr("目标主机:"), udpTargetHost_);
    udpForm->addRow(tr("目标端口:"), udpTargetPort_);

    auto* serialBox = new QGroupBox(tr("串口 (数传 / RS485 / RS422 / CAN桥)"), this);
    auto* serialForm = new QFormLayout(serialBox);
    serialPort_ = new QComboBox(this);
    const auto ports = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo& info : ports) {
        serialPort_->addItem(info.portName() + QStringLiteral("  ") + info.description(),
                             info.portName());
    }
    if (serialPort_->count() == 0)
        serialPort_->addItem(tr("(未检测到串口)"), QString());
    serialBaud_ = new QSpinBox(this);
    serialBaud_->setRange(1200, 3000000);
    serialBaud_->setValue(57600);
    serialForm->addRow(tr("端口:"), serialPort_);
    serialForm->addRow(tr("波特率:"), serialBaud_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(udpBox);
    layout->addWidget(serialBox);
    layout->addWidget(buttons);
}

} // namespace skygcs
