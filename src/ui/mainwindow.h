#pragma once
// ============================================================================
// SkyGCS  地面站主窗口
// 标签页: 遥测监控 / 飞行指令 / 载荷串口控制台 / 消息检查器
// 链路: UDP (SITL/真机) + 串口 (数传/RS485/RS422)
// 仿真: 内置 PX4 仿真器 (注入模式 / UDP 服务模式)
// ============================================================================
#include <QMainWindow>

#include "../comm/mavlinkendpoint.h"
#include "../sim/flightsim.h"

class QTabWidget;
class QLabel;
class QPlainTextEdit;
class QPushButton;

namespace skygcs {

class TelemetryPanel;
class CommandPanel;
class SerialConsole;
class MessageInspector;

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onAddUdpLink();
    void onAddSerialLink();
    void onStartSimInject();
    void onStartSimUdp();
    void onStopSim();
    void onLog(const QString& text);
    void onStatustext(int severity, const QString& text);
    void onConnectionChanged(bool online);

private:
    void appendLog(const QString& text);

    MavlinkEndpoint* endpoint_ = nullptr;
    FlightSim* sim_ = nullptr;

    QTabWidget* tabs_ = nullptr;
    TelemetryPanel* telemetry_ = nullptr;
    CommandPanel* commands_ = nullptr;
    SerialConsole* serialConsole_ = nullptr;
    MessageInspector* inspector_ = nullptr;

    QLabel* statusLink_ = nullptr;
    QLabel* statusVehicle_ = nullptr;
    QPlainTextEdit* logView_ = nullptr;
    QPushButton* btnSimUdp_ = nullptr;
};

} // namespace skygcs
