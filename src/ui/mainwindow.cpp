#include "mainwindow.h"

#include <QAction>
#include <QComboBox>
#include <QDockWidget>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QSplitter>
#include <QStatusBar>
#include <QTabWidget>
#include <QTime>
#include <QToolBar>

#include "../comm/seriallink.h"
#include "../comm/udplink.h"
#include "commandpanel.h"
#include "linkdialog.h"
#include "messageinspector.h"
#include "missionpanel.h"
#include "serialconsole.h"
#include "telemetrychart.h"
#include "telemetrypanel.h"

namespace skygcs {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle(tr("SkyGCS 无人机地面站 (MAVLink / QT / C++)"));
    resize(1280, 820);

    endpoint_ = new MavlinkEndpoint(this);
    sim_ = new FlightSim(this);
    // 仿真注入模式: 消息直接进入协议引擎
    connect(sim_, &FlightSim::messageReady, endpoint_, &MavlinkEndpoint::injectMessage);
    connect(sim_, &FlightSim::logMessage, this, &MainWindow::onLog);

    // ---- 中央标签页 ----
    tabs_ = new QTabWidget(this);
    telemetry_ = new TelemetryPanel(endpoint_->vehicle(), this);
    auto* teleChart = new TelemetryChart(endpoint_->vehicle(), this);
    auto* teleSplit = new QSplitter(Qt::Vertical, this);
    teleSplit->addWidget(telemetry_);
    teleSplit->addWidget(teleChart);
    teleSplit->setStretchFactor(0, 3);
    teleSplit->setStretchFactor(1, 4);
    commands_ = new CommandPanel(endpoint_, this);
    mission_ = new MissionPanel(endpoint_, this);
    serialConsole_ = new SerialConsole(this);
    inspector_ = new MessageInspector(endpoint_, this);
    tabs_->addTab(teleSplit, tr("遥测监控"));
    tabs_->addTab(commands_, tr("飞行指令"));
    tabs_->addTab(mission_, tr("任务规划"));
    tabs_->addTab(serialConsole_, tr("载荷串口控制台"));
    tabs_->addTab(inspector_, tr("消息检查器"));
    setCentralWidget(tabs_);

    // ---- 底部日志 ----
    logView_ = new QPlainTextEdit(this);
    logView_->setReadOnly(true);
    logView_->setMaximumBlockCount(1000);
    logView_->setStyleSheet("QPlainTextEdit{background:#10131a;color:#b8c4d6;"
                            "font-family:Consolas,monospace;font-size:12px;}");
    auto* dock = new QDockWidget(tr("系统日志"), this);
    dock->setWidget(logView_);
    dock->setAllowedAreas(Qt::BottomDockWidgetArea);
    addDockWidget(Qt::BottomDockWidgetArea, dock);

    // ---- 菜单/工具栏 ----
    auto* menuLink = menuBar()->addMenu(tr("链路(&L)"));
    auto* actUdp = menuLink->addAction(tr("新建 UDP 链路..."));
    auto* actSerial = menuLink->addAction(tr("新建串口链路..."));
    menuLink->addSeparator();
    auto* actQuit = menuLink->addAction(tr("退出"));

    auto* menuSim = menuBar()->addMenu(tr("仿真(&S)"));
    auto* actSimInject = menuSim->addAction(tr("启动内置仿真(注入)"));
    auto* actSimUdp = menuSim->addAction(tr("启动内置仿真(UDP服务)"));
    menuSim->addSeparator();
    auto* actSimStop = menuSim->addAction(tr("停止仿真"));

    auto* tb = addToolBar(tr("主工具栏"));
    tb->setMovable(false);
    btnSimUdp_ = new QPushButton(tr("▶ 启动内置仿真 (UDP)"), this);
    btnSimUdp_->setStyleSheet("QPushButton{background:#16a085;color:white;font-weight:600;}");
    tb->addWidget(btnSimUdp_);
    tb->addAction(actSimInject);

    connect(actUdp, &QAction::triggered, this, &MainWindow::onAddUdpLink);
    connect(actSerial, &QAction::triggered, this, &MainWindow::onAddSerialLink);
    connect(actQuit, &QAction::triggered, this, &QWidget::close);
    connect(actSimInject, &QAction::triggered, this, &MainWindow::onStartSimInject);
    connect(actSimUdp, &QAction::triggered, this, &MainWindow::onStartSimUdp);
    connect(actSimStop, &QAction::triggered, this, &MainWindow::onStopSim);
    connect(btnSimUdp_, &QPushButton::clicked, this, &MainWindow::onStartSimUdp);

    // ---- 状态栏 ----
    statusLink_ = new QLabel(tr("无链路"), this);
    statusVehicle_ = new QLabel(tr("飞行器离线"), this);
    statusBar()->addWidget(statusLink_, 1);
    statusBar()->addPermanentWidget(statusVehicle_);

    // ---- 引擎信号 ----
    connect(endpoint_, &MavlinkEndpoint::logMessage, this, &MainWindow::onLog);
    connect(endpoint_, &MavlinkEndpoint::statustext, this, &MainWindow::onStatustext);
    connect(endpoint_, &MavlinkEndpoint::vehicleOnline, this,
            &MainWindow::onConnectionChanged);

    onLog(tr("SkyGCS 就绪 — 使用「启动内置仿真」可零硬件体验完整链路"));
}

// ---------------------------------------------------------------------------
void MainWindow::onAddUdpLink()
{
    LinkDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted)
        return;
    auto* link = new UdpLink(this);
    UdpLink::Config cfg;
    cfg.localPort = static_cast<quint16>(dlg.udpLocalPort()->text().toUShort());
    cfg.remoteHost = dlg.udpTargetHost()->text();
    cfg.remotePort = static_cast<quint16>(dlg.udpTargetPort()->text().toUShort());
    link->setConfig(cfg);
    if (!link->open()) {
        QMessageBox::warning(this, tr("链路错误"), tr("UDP 绑定失败, 端口可能被占用"));
        link->deleteLater();
        return;
    }
    endpoint_->addLink(link);
    connect(link, &LinkInterface::connectedChanged, this,
            [this, link](bool ok) { statusLink_->setText(ok ? link->detail() : tr("链路断开")); });
    statusLink_->setText(link->detail());
    onLog(tr("[链路] UDP %1:%2 (目标 %3:%4)")
              .arg(cfg.localPort).arg(cfg.remoteHost).arg(cfg.remotePort));
    statusBar()->showMessage(tr("UDP 链路已建立"), 3000);
}

void MainWindow::onAddSerialLink()
{
    LinkDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted)
        return;
    const QString portName = dlg.serialPort()->currentData().toString();
    if (portName.isEmpty()) {
        QMessageBox::warning(this, tr("链路错误"), tr("未选择有效串口"));
        return;
    }
    auto* link = new SerialLink(this);
    SerialLink::Config cfg;
    cfg.portName = portName;
    cfg.baudRate = dlg.serialBaud()->value();
    link->setConfig(cfg);
    if (!link->open()) {
        QMessageBox::warning(this, tr("链路错误"),
                             tr("串口 %1 打开失败 (可能被占用)").arg(portName));
        link->deleteLater();
        return;
    }
    endpoint_->addLink(link);
    serialConsole_->attachLink(link);
    connect(link, &LinkInterface::connectedChanged, this,
            [this, link](bool ok) { statusLink_->setText(ok ? link->detail() : tr("串口断开")); });
    statusLink_->setText(link->detail());
    onLog(tr("[链路] 串口 %1 @ %2").arg(cfg.portName).arg(cfg.baudRate));
    statusBar()->showMessage(tr("串口链路已建立"), 3000);
}

void MainWindow::onStartSimInject()
{
    if (sim_->running()) {
        onLog(tr("[仿真] 仿真已在运行, 先停止"));
        return;
    }
    sim_->setHome(24.51, 117.65, 30.0);   // 漳州
    sim_->startInject();
    onLog(tr("[仿真] 注入模式已启动 (遥测直通协议引擎)"));
}

void MainWindow::onStartSimUdp()
{
    if (sim_->running()) {
        onLog(tr("[仿真] 仿真已在运行, 先停止"));
        return;
    }
    // 地面站监听 14550, 仿真器遥测发往 14550
    auto* link = new UdpLink(this);
    UdpLink::Config cfg;
    cfg.localPort = 14550;
    cfg.remoteHost = QStringLiteral("127.0.0.1");
    cfg.remotePort = 14550;
    link->setConfig(cfg);
    if (!link->open()) {
        QMessageBox::warning(this, tr("链路错误"),
                             tr("14550 端口被占用, 无法启动仿真链路"));
        link->deleteLater();
        return;
    }
    endpoint_->addLink(link);
    connect(link, &LinkInterface::connectedChanged, this,
            [this, link](bool ok) { statusLink_->setText(ok ? link->detail() : tr("链路断开")); });
    statusLink_->setText(link->detail());

    sim_->setHome(24.51, 117.65, 30.0);   // 漳州
    if (!sim_->startUdpServer(14550)) {
        QMessageBox::warning(this, tr("仿真错误"), tr("仿真 UDP 服务启动失败"));
        return;
    }
    onLog(tr("[仿真] UDP 服务模式已启动: 遥测 → 127.0.0.1:14550"));
}

void MainWindow::onStopSim()
{
    sim_->stop();
    onLog(tr("[仿真] 已停止"));
}

void MainWindow::onLog(const QString& text)
{
    logView_->appendPlainText(QTime::currentTime().toString(QStringLiteral("[HH:mm:ss] "))
                              + text);
}

void MainWindow::onStatustext(int severity, const QString& text)
{
    const char* color = "#8a93a6";
    switch (severity) {
    case mav::SEVERITY_EMERGENCY:
    case mav::SEVERITY_ALERT:
    case mav::SEVERITY_CRITICAL: color = "#ff5c5c"; break;
    case mav::SEVERITY_ERROR: color = "#ff9a8a"; break;
    case mav::SEVERITY_WARNING: color = "#f5c542"; break;
    case mav::SEVERITY_INFO: color = "#9fe8a8"; break;
    default: break;
    }
    logView_->appendHtml(QStringLiteral("<span style='color:%1'>[飞控] %2</span>")
                             .arg(color, text.toHtmlEscaped()));
}

void MainWindow::onConnectionChanged(bool online)
{
    statusVehicle_->setText(online ? tr("飞行器在线") : tr("飞行器离线"));
    statusVehicle_->setStyleSheet(online ? "color:#2ecc71;font-weight:600;"
                                         : "color:#e67e22;");
}

} // namespace skygcs
