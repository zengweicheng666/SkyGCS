// ============================================================================
// SkyGCS  端到端集成测试 (Qt)
// 链路: 内置仿真器 (UDP 服务) → UdpLink → MavlinkEndpoint → VehicleState
// 验证: 1) 遥测连通 (heartbeat → online, ATTITUDE/GLOBAL_POSITION_INT 刷新)
//       2) 指令下发 COMMAND_LONG(ARM) → 仿真器 ACK → endpoint 收到
//       3) STATUSTEXT 事件透传
// 运行: build 目录下 ./test_integration.exe
// ============================================================================
#include <QCoreApplication>
#include <QTimer>
#include <cstdio>

#include "../src/comm/mavlinkendpoint.h"
#include "../src/comm/udplink.h"
#include "../src/sim/flightsim.h"

using namespace skygcs;

static int g_fail = 0;
static void check(bool ok, const char* what)
{
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        ++g_fail;
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    auto* endpoint = new MavlinkEndpoint;
    auto* sim = new FlightSim;

    auto* link = new UdpLink;
    UdpLink::Config cfg;
    cfg.localPort = 14550;
    link->setConfig(cfg);
    if (!link->open()) {
        std::printf("[FAIL] UDP 14550 绑定失败 (可能被占用)\n");
        return 1;
    }
    endpoint->addLink(link);

    bool gotOnline = false;
    bool gotAck = false;
    bool gotStatus = false;
    bool gotAttitude = false;
    bool gotPos = false;

    QObject::connect(endpoint, &MavlinkEndpoint::vehicleOnline, [&](bool online) {
        gotOnline = gotOnline || online;
    });
    QObject::connect(endpoint, &MavlinkEndpoint::commandAck, [&](uint16_t cmd, uint8_t res, int, int) {
        if (cmd == mav::CMD_COMPONENT_ARM_DISARM && res == mav::RESULT_ACCEPTED)
            gotAck = true;
    });
    QObject::connect(endpoint, &MavlinkEndpoint::statustext, [&](int, const QString&) {
        gotStatus = true;
    });
    QObject::connect(endpoint, &MavlinkEndpoint::messageReceived, [&](const MavMessage& msg) {
        if (msg.msgid == MAV_MSG_ID_ATTITUDE)
            gotAttitude = true;
        if (msg.msgid == MAV_MSG_ID_GLOBAL_POSITION_INT)
            gotPos = true;
    });

    sim->setHome(24.51, 117.65, 30.0);
    if (!sim->startUdpServer(14550)) {
        std::printf("[FAIL] 仿真器 UDP 服务启动失败\n");
        return 1;
    }

    QTimer::singleShot(2500, [&]() {
        endpoint->armDisarm(true);   // 2.5s 后下发解锁, 验证 ACK 闭环
    });
    QTimer::singleShot(3000, [&]() {
        endpoint->takeoff(8.0f);     // 3.0s 后起飞, 验证高度爬升
    });
    // 每秒打印轨迹 (验证物理模型: 解锁→起飞爬升)
    int traceSec = 0;
    auto* trace = new QTimer;
    QObject::connect(trace, &QTimer::timeout, [&]() {
        ++traceSec;
        const VehicleState* v = endpoint->vehicle();
        std::printf("  [轨迹] t=%2ds armed=%d alt=%5.1fm climb=%+5.1f mode=%s\n",
                    traceSec, v->armed() ? 1 : 0, v->relAlt(),
                    v->climb(), qPrintable(v->modeName()));
    });
    trace->start(1000);
    QTimer::singleShot(9000, [&]() {
        const VehicleState* v = endpoint->vehicle();
        check(v->isOnline(), "心跳 → 飞行器在线");
        check(gotOnline, "vehicleOnline 信号触发");
        check(gotAttitude, "收到 ATTITUDE 遥测");
        check(gotPos, "收到 GLOBAL_POSITION_INT 遥测");
        check(v->lat() > 24.0 && v->lon() > 117.0, "经纬度聚合正确 (漳州附近)");
        check(gotAck, "ARM 指令收到 ACK (ACCEPTED)");
        check(gotStatus, "STATUSTEXT 事件透传");
        check(sim->running(), "仿真器运行中");
        check(v->relAlt() > 5.0, "起飞后高度爬升 (>5m)");
        std::printf("\n%s (%d 项)\n", g_fail ? "存在失败项" : "全部通过", g_fail);
        app.exit(g_fail ? 1 : 0);
    });

    return app.exec();
}
