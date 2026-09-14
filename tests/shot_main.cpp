// ============================================================================
// SkyGCS  UI 离屏渲染截图工具 (验证/文档用)
// 启动主窗口 → 程序化启动内置仿真 (UDP 模式) → 等待遥测 → grab 保存 PNG
// 运行: build 目录 ./shot_main.exe [输出路径]
// 注意: offscreen 平台无默认字体数据库, 需显式加载系统中文字体
// ============================================================================
#include <QApplication>
#include <QFont>
#include <QFontDatabase>
#include <QPixmap>
#include <QSplitter>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <cstdio>

#include "../src/comm/mavlinkendpoint.h"
#include "../src/comm/udplink.h"
#include "../src/sim/flightsim.h"
#include "../src/ui/commandpanel.h"
#include "../src/ui/flightlogpanel.h"
#include "../src/ui/messageinspector.h"
#include "../src/ui/missionpanel.h"
#include "../src/ui/parameterpanel.h"
#include "../src/ui/telemetrychart.h"
#include "../src/ui/telemetrypanel.h"

using namespace skygcs;

static void loadChineseFont(QApplication& app)
{
    // offscreen 平台不加载系统字体 → 中文会渲染为方块, 显式注册
    const QStringList candidates = {
        QStringLiteral("C:/Windows/Fonts/msyh.ttc"),   // 微软雅黑
        QStringLiteral("C:/Windows/Fonts/simhei.ttf"), // 黑体
        QStringLiteral("C:/Windows/Fonts/simsun.ttc"), // 宋体
    };
    QString family;
    for (const QString& path : candidates) {
        const int id = QFontDatabase::addApplicationFont(path);
        if (id >= 0) {
            family = QFontDatabase::applicationFontFamilies(id).value(0);
            if (!family.isEmpty())
                break;
        }
    }
    QFont font(family.isEmpty() ? QStringLiteral("Microsoft YaHei") : family);
    font.setPixelSize(13);
    app.setFont(font);
    std::printf("[font] %s\n", qPrintable(family));
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    loadChineseFont(app);

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
    // (仿真在 2s 后启动, 以便先截"未连接"初始界面)

    // 独立窗口组合: 遥测(数值+曲线) / 飞行指令 / 任务规划 / 参数管理 / 飞行日志 / 消息检查器
    QWidget host;
    host.setWindowTitle(QStringLiteral("SkyGCS 无人机地面站 (MAVLink / QT / C++)"));
    host.resize(1295, 857);
    auto* tabs = new QTabWidget(&host);
    auto* teleSplit = new QSplitter(Qt::Vertical, tabs);
    auto* telemetry = new TelemetryPanel(endpoint->vehicle(), teleSplit);
    auto* teleChart = new TelemetryChart(endpoint->vehicle(), teleSplit);
    teleSplit->addWidget(telemetry);
    teleSplit->addWidget(teleChart);
    teleSplit->setStretchFactor(0, 3);
    teleSplit->setStretchFactor(1, 4);
    tabs->addTab(teleSplit, QStringLiteral("遥测监控"));
    tabs->addTab(new CommandPanel(endpoint, tabs), QStringLiteral("飞行指令"));
    auto* mission = new MissionPanel(endpoint, tabs);
    tabs->addTab(mission, QStringLiteral("任务规划"));
    auto* params = new ParameterPanel(endpoint, tabs);
    tabs->addTab(params, QStringLiteral("参数管理"));
    tabs->addTab(new FlightLogPanel(endpoint, tabs), QStringLiteral("飞行日志"));
    auto* msgList = new QWidget(tabs);
    auto* ml = new QVBoxLayout(msgList);
    ml->addWidget(new MessageInspector(endpoint, msgList));
    tabs->addTab(msgList, QStringLiteral("消息检查器"));
    auto* layout = new QVBoxLayout(&host);
    layout->addWidget(tabs);
    host.show();

    const QString outMain    = (argc > 4) ? QString::fromLocal8Bit(argv[4])
                                          : QString();
    const QString outParam   = (argc > 1) ? QString::fromLocal8Bit(argv[1])
                                          : QStringLiteral("docs/screenshot_live.png");
    const QString outMission = (argc > 2) ? QString::fromLocal8Bit(argv[2]) : QString();
    const QString outLive    = (argc > 3) ? QString::fromLocal8Bit(argv[3]) : QString();

    // 初始界面 (未启动仿真, 飞行器离线)
    QTimer::singleShot(800, [&]() {
        if (!outMain.isEmpty()) {
            const QPixmap pm = host.grab();
            const bool ok = pm.save(outMain);
            std::printf("%s %s (%dx%d)\n", ok ? "SAVED" : "FAIL", qPrintable(outMain),
                        pm.width(), pm.height());
        }
    });
    // 启动仿真 (让遥测/任务/参数有实际数据)
    QTimer::singleShot(2000, [&]() {
        sim->startUdpServer(14550);
    });
    QTimer::singleShot(3000, [&]() {
        // 下发解锁 + 起飞, 让姿态/高度有实际变化
        endpoint->armDisarm(true);
        endpoint->takeoff(8.0f);
    });
    QTimer::singleShot(10500, [&]() {
        // 上传 3 航点任务并开始 (与 test_integration 同款)
        QVector<MavlinkEndpoint::MissionItem> items;
        const double hLat = 24.51, hLon = 117.65;
        const double dLat1 = 60.0 / 111320.0;
        const double dLon1 = 60.0 / (111320.0 * std::cos(hLat * M_PI / 180.0));
        MavlinkEndpoint::MissionItem it;
        it.frame = mav::FRAME_GLOBAL_RELATIVE_ALT_INT;
        it.z = 8.0f;
        it.x = static_cast<int32_t>((hLat + dLat1) * 1e7);
        it.y = static_cast<int32_t>(hLon * 1e7);
        items.append(it);
        it.x = static_cast<int32_t>((hLat + dLat1) * 1e7);
        it.y = static_cast<int32_t>((hLon + dLon1) * 1e7);
        items.append(it);
        it.x = static_cast<int32_t>((hLat + 2 * dLat1) * 1e7);
        items.append(it);
        endpoint->uploadMission(items);
    });
    QTimer::singleShot(15000, [&]() {
        endpoint->startMission();
    });
    QTimer::singleShot(17000, [&]() {
        // 读取参数列表 (参数管理演示)
        endpoint->requestParamList();
    });
    QTimer::singleShot(19500, [&]() {
        // 修改巡航速度参数, 等待飞控回传确认
        endpoint->setParam(QStringLiteral("MPC_XY_CRUISE"), 8.5f);
    });
    QTimer::singleShot(22000, [&]() {
        // 切换到参数管理页, 展示参数表 (含确认状态)
        tabs->setCurrentWidget(params);
    });
    QTimer::singleShot(26000, [&]() {
        const QPixmap pm = host.grab();
        const bool ok = pm.save(outParam);
        std::printf("%s %s (%dx%d)\n", ok ? "SAVED" : "FAIL", qPrintable(outParam),
                    pm.width(), pm.height());
        const VehicleState* v = endpoint->vehicle();
        std::printf("vehicle online=%d armed=%d altRel=%.1f mode=%s cur=%d reached=%d\n",
                    v->isOnline(), v->armed() ? 1 : 0, v->relAlt(),
                    qPrintable(v->modeName()), v->missionCurrent(), v->missionReached());
        float cruise = -1;
        v->paramValue(QStringLiteral("MPC_XY_CRUISE"), cruise);
        std::printf("params=%d MPC_XY_CRUISE=%.2f\n", v->paramCount(), cruise);
        // 切换到任务规划页 (展示任务进度)
        if (!outMission.isEmpty())
            tabs->setCurrentWidget(mission);
        else
            app.exit(ok ? 0 : 1);
    });
    QTimer::singleShot(30000, [&]() {
        if (!outMission.isEmpty()) {
            const QPixmap pm = host.grab();
            const bool ok = pm.save(outMission);
            std::printf("%s %s (%dx%d)\n", ok ? "SAVED" : "FAIL", qPrintable(outMission),
                        pm.width(), pm.height());
        }
        // 切回遥测页
        if (!outLive.isEmpty())
            tabs->setCurrentWidget(teleSplit);
    });
    QTimer::singleShot(34000, [&]() {
        if (!outLive.isEmpty()) {
            const QPixmap pm = host.grab();
            const bool ok = pm.save(outLive);
            std::printf("%s %s (%dx%d)\n", ok ? "SAVED" : "FAIL", qPrintable(outLive),
                        pm.width(), pm.height());
        }
        app.exit(0);
    });
    return app.exec();
}
