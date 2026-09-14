// ============================================================================
// SkyGCS  UI 离屏渲染截图工具 (验证/文档用)
// 启动主窗口 → 程序化启动内置仿真 (UDP 模式) → 等待遥测 → grab 保存 PNG
// 运行: build 目录 ./shot_main.exe [输出路径]
// ============================================================================
#include <QApplication>
#include <QPixmap>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>

#include "../src/comm/mavlinkendpoint.h"
#include "../src/comm/udplink.h"
#include "../src/sim/flightsim.h"
#include "../src/ui/commandpanel.h"
#include "../src/ui/messageinspector.h"
#include "../src/ui/telemetrypanel.h"

using namespace skygcs;

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    auto* endpoint = new MavlinkEndpoint;
    auto* sim = new FlightSim;
    QObject::connect(sim, &FlightSim::messageReady, endpoint, &MavlinkEndpoint::injectMessage);

    auto* link = new UdpLink;
    UdpLink::Config cfg;
    cfg.localPort = 14550;
    link->setConfig(cfg);
    endpoint->addLink(link);
    if (!link->open()) {
        std::printf("FAIL: UDP 14550 绑定失败\n");
        return 1;
    }
    sim->setHome(24.51, 117.65, 30.0);
    sim->startUdpServer(14550);

    // 让 MainWindow 也持有同一 endpoint 无法直接注入, 改用独立窗口组合:
    // 直接构造 TelemetryPanel+CommandPanel 用 endpoint 展示
    QWidget host;
    host.setWindowTitle(QStringLiteral("SkyGCS 无人机地面站 (MAVLink / QT / C++)"));
    host.resize(1295, 857);
    auto* tabs = new QTabWidget(&host);
    tabs->addTab(new TelemetryPanel(endpoint->vehicle(), tabs), QStringLiteral("遥测监控"));
    tabs->addTab(new CommandPanel(endpoint, tabs), QStringLiteral("飞行指令"));
    auto* msgList = new QWidget(tabs);
    auto* ml = new QVBoxLayout(msgList);
    ml->addWidget(new MessageInspector(endpoint, msgList));
    tabs->addTab(msgList, QStringLiteral("消息检查器"));
    auto* layout = new QVBoxLayout(&host);
    layout->addWidget(tabs);
    host.show();

    QTimer::singleShot(3000, [&]() {
        // 下发解锁 + 起飞, 让姿态/高度有实际变化
        endpoint->armDisarm(true);
        endpoint->takeoff(8.0f);
    });
    QTimer::singleShot(9000, [&]() {
        const QString out = (argc > 1) ? QString::fromLocal8Bit(argv[1])
                                       : QStringLiteral("docs/screenshot_live.png");
        const QPixmap pm = host.grab();
        const bool ok = pm.save(out);
        std::printf("%s %s (%dx%d)\n", ok ? "SAVED" : "FAIL", qPrintable(out),
                    pm.width(), pm.height());
        std::printf("vehicle online=%d armed=%d altRel=%.1f mode=%s\n",
                    endpoint->vehicle()->isOnline(),
                    endpoint->vehicle()->armed() ? 1 : 0,
                    endpoint->vehicle()->relAlt(),
                    qPrintable(endpoint->vehicle()->modeName()));
        app.exit(ok ? 0 : 1);
    });
    return app.exec();
}
