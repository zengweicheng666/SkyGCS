#include "commandpanel.h"

#include <QComboBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include "../mavlink/mavlink_names.h"

namespace skygcs {

CommandPanel::CommandPanel(MavlinkEndpoint* endpoint, QWidget* parent)
    : QWidget(parent)
    , endpoint_(endpoint)
{
    auto* cmdBox = new QGroupBox(tr("飞行指令 (COMMAND_LONG + ACK)"), this);

    btnArm_ = new QPushButton(tr("解锁 ARM"), this);
    btnArm_->setStyleSheet("QPushButton{background:#27ae60;color:white;font-weight:600;}"
                           "QPushButton:hover{background:#2ecc71;}");
    btnDisarm_ = new QPushButton(tr("上锁 DISARM"), this);
    btnDisarm_->setStyleSheet("QPushButton{background:#c0392b;color:white;font-weight:600;}"
                              "QPushButton:hover{background:#e74c3c;}");
    btnTakeoff_ = new QPushButton(tr("起飞 TAKEOFF"), this);
    btnLand_ = new QPushButton(tr("降落 LAND"), this);
    btnRtl_ = new QPushButton(tr("返航 RTL"), this);

    comboMode_ = new QComboBox(this);
    comboMode_->addItem(tr("手动 Manual"), mav::PX4_MODE_MANUAL);
    comboMode_->addItem(tr("定高 PosCtl"), mav::PX4_MODE_POSCTL);
    comboMode_->addItem(tr("自动·盘旋 Loiter"), mav::PX4_MODE_AUTO * 256 + mav::PX4_AUTO_LOITER);
    comboMode_->addItem(tr("自动·任务 Mission"), mav::PX4_MODE_AUTO * 256 + mav::PX4_AUTO_MISSION);
    comboMode_->addItem(tr("自动·返航 RTL"), mav::PX4_MODE_AUTO * 256 + mav::PX4_AUTO_RTL);
    comboMode_->addItem(tr("自动·降落 Land"), mav::PX4_MODE_AUTO * 256 + mav::PX4_AUTO_LAND);

    spinAlt_ = new QSpinBox(this);
    spinAlt_->setRange(1, 120);
    spinAlt_->setValue(5);
    spinAlt_->setSuffix(tr(" m"));

    auto* grid = new QGridLayout(cmdBox);
    grid->addWidget(btnArm_, 0, 0);
    grid->addWidget(btnDisarm_, 0, 1);
    grid->addWidget(btnTakeoff_, 1, 0);
    grid->addWidget(btnLand_, 1, 1);
    grid->addWidget(btnRtl_, 2, 0);
    grid->addWidget(new QLabel(tr("起飞高度:"), this), 2, 1);
    grid->addWidget(spinAlt_, 2, 2);

    auto* modeBox = new QGroupBox(tr("飞行模式 (SET_MODE)"), this);
    auto* modeLayout = new QHBoxLayout(modeBox);
    modeLayout->addWidget(new QLabel(tr("目标模式:"), this));
    modeLayout->addWidget(comboMode_, 1);

    logView_ = new QPlainTextEdit(this);
    logView_->setReadOnly(true);
    logView_->setMaximumBlockCount(500);
    logView_->setStyleSheet("QPlainTextEdit{background:#14181f;color:#9fe8a8;"
                            "font-family:Consolas,monospace;font-size:12px;}");

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(cmdBox);
    layout->addWidget(modeBox);
    layout->addWidget(new QLabel(tr("指令/ACK 日志"), this));
    layout->addWidget(logView_, 1);

    connect(btnArm_, &QPushButton::clicked, this, &CommandPanel::onArm);
    connect(btnDisarm_, &QPushButton::clicked, this, &CommandPanel::onDisarm);
    connect(btnTakeoff_, &QPushButton::clicked, this, &CommandPanel::onTakeoff);
    connect(btnLand_, &QPushButton::clicked, this, &CommandPanel::onLand);
    connect(btnRtl_, &QPushButton::clicked, this, &CommandPanel::onRtl);
    connect(comboMode_, qOverload<int>(&QComboBox::activated), this, &CommandPanel::onModeChanged);

    connect(endpoint_, &MavlinkEndpoint::commandAck, this, &CommandPanel::onAck);
    connect(endpoint_, &MavlinkEndpoint::logMessage, this, &CommandPanel::onLog);
}

void CommandPanel::onArm()
{
    endpoint_->armDisarm(true);
    onLog(tr("[UI] 下发解锁指令"));
}

void CommandPanel::onDisarm()
{
    endpoint_->armDisarm(false);
    onLog(tr("[UI] 下发放弃/上锁指令"));
}

void CommandPanel::onTakeoff()
{
    endpoint_->takeoff(static_cast<float>(spinAlt_->value()));
    onLog(tr("[UI] 下发起飞指令 (目标 %1 m)").arg(spinAlt_->value()));
}

void CommandPanel::onLand()
{
    endpoint_->land();
    onLog(tr("[UI] 下发降落指令"));
}

void CommandPanel::onRtl()
{
    endpoint_->rtl();
    onLog(tr("[UI] 下发返航指令"));
}

void CommandPanel::onModeChanged(int index)
{
    const int v = comboMode_->itemData(index).toInt();
    if (v < 256) {
        endpoint_->sendSetMode(static_cast<uint32_t>(v), 0);
        onLog(tr("[UI] 下发 SET_MODE main=%1").arg(v));
    } else {
        endpoint_->sendSetMode(static_cast<uint32_t>(v / 256), static_cast<uint32_t>(v % 256));
        onLog(tr("[UI] 下发 SET_MODE main=%1 sub=%2").arg(v / 256).arg(v % 256));
    }
}

void CommandPanel::onAck(uint16_t command, uint8_t result, int, int)
{
    QString name = mav::commandName(command);
    QString res = mav::commandResultName(result);
    const QString color = (result == mav::RESULT_ACCEPTED) ? "#9fe8a8" : "#ff9a8a";
    logView_->appendHtml(QStringLiteral("<span style='color:%1'>[ACK] %2 (0x%3) → %4</span>")
                             .arg(color, name)
                             .arg(command, 4, 16, QChar('0'))
                             .arg(res));
}

void CommandPanel::onLog(const QString& text)
{
    logView_->appendPlainText(text);
}

} // namespace skygcs
