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
#include <cmath>
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
    bool gotStatusChinese = false;
    bool gotAttitude = false;
    bool gotPos = false;
    bool gotMissionReached = false;
    int missionReachedSeq = -1;
    // 参数
    bool gotParamList = false;
    float paramCruiseValue = -1.0f;

    QObject::connect(endpoint, &MavlinkEndpoint::vehicleOnline, [&](bool online) {
        gotOnline = gotOnline || online;
    });
    QObject::connect(endpoint->vehicle(), &VehicleState::paramChanged,
                     [&](const QString& name, float value) {
        if (name == QStringLiteral("MPC_XY_CRUISE")) {
            paramCruiseValue = value;
            gotParamList = endpoint->vehicle()->paramCount() >= 8;
        }
    });
    QObject::connect(endpoint, &MavlinkEndpoint::commandAck, [&](uint16_t cmd, uint8_t res, int, int) {
        if (cmd == mav::CMD_COMPONENT_ARM_DISARM && res == mav::RESULT_ACCEPTED)
            gotAck = true;
    });
    QObject::connect(endpoint, &MavlinkEndpoint::statustext, [&](int, const QString& t) {
        gotStatus = true;
        if (t.contains(QStringLiteral("SkyGCS 仿真 UDP 服务启动")))
            gotStatusChinese = true;   // UTF-8 中文往返 (防 fromLatin1 回归)
    });
    QObject::connect(endpoint, &MavlinkEndpoint::messageReceived, [&](const MavMessage& msg) {
        if (msg.msgid == MAV_MSG_ID_ATTITUDE)
            gotAttitude = true;
        if (msg.msgid == MAV_MSG_ID_GLOBAL_POSITION_INT)
            gotPos = true;
        if (msg.msgid == MAV_MSG_ID_MISSION_ITEM_REACHED) {
            MissionItemReachedMsg r;
            if (unpackMissionItemReached(msg, r)) {
                gotMissionReached = true;
                missionReachedSeq = r.seq;
            }
        }
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
    double maxLat = 24.51, maxLon = 117.65;
    auto* trace = new QTimer;
    QObject::connect(trace, &QTimer::timeout, [&]() {
        ++traceSec;
        const VehicleState* v = endpoint->vehicle();
        maxLat = std::max(maxLat, v->lat());
        maxLon = std::max(maxLon, v->lon());
        std::printf("  [轨迹] t=%2ds armed=%d alt=%5.1fm climb=%+5.1f mode=%s cur=%d reached=%d\n",
                    traceSec, v->armed() ? 1 : 0, v->relAlt(),
                    v->climb(), qPrintable(v->modeName()),
                    v->missionCurrent(), v->missionReached());
        std::fflush(stdout);
    });
    trace->start(1000);

    // ---- 阶段2: 航点任务 ----
    QTimer::singleShot(10000, [&]() {
        QVector<MavlinkEndpoint::MissionItem> items;
        const double hLat = 24.51, hLon = 117.65;
        // 航点1: 北 30m; 航点2: 北 30m 东 30m; 航点3: 北 60m 东 30m
        const double dLat1 = 30.0 / 111320.0;
        const double dLon1 = 30.0 / (111320.0 * std::cos(hLat * M_PI / 180.0));
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
    QTimer::singleShot(14500, [&]() {
        endpoint->startMission();    // 上传完成后开始任务
    });

    // ---- 阶段3: 参数管理 ----
    QTimer::singleShot(16500, [&]() {
        endpoint->requestParamList();   // 请求 8 个参数
    });
    QTimer::singleShot(18000, [&]() {
        endpoint->setParam(QStringLiteral("MPC_XY_CRUISE"), 8.5f);   // 修改巡航速度
    });

    QTimer::singleShot(52000, [&]() {
        const VehicleState* v = endpoint->vehicle();
        check(v->isOnline(), "心跳 → 飞行器在线");
        check(gotOnline, "vehicleOnline 信号触发");
        check(gotAttitude, "收到 ATTITUDE 遥测");
        check(gotPos, "收到 GLOBAL_POSITION_INT 遥测");
        check(v->lat() > 24.0 && v->lon() > 117.0, "经纬度聚合正确 (漳州附近)");
        check(gotAck, "ARM 指令收到 ACK (ACCEPTED)");
        check(gotStatus, "STATUSTEXT 事件透传");
        check(gotStatusChinese, "STATUSTEXT 中文往返 (UTF-8 编码)");
        check(sim->running(), "仿真器运行中");
        check(v->relAlt() > 5.0, "起飞后高度爬升 (>5m)");
        check(v->missionUploaded(), "任务上传被飞控确认 (MISSION_ACK=ACCEPTED)");
        check(gotMissionReached, "收到 MISSION_ITEM_REACHED 事件");
        check(v->missionReached() >= 1, "至少到达 2 个航点 (reached>=1)");
        // 任务期间峰值位置应深入东北方向 (RTL 后已回到原点)
        check(maxLat > 24.5102 && maxLon > 117.6502,
              "任务期间位置向航点方向移动 (峰值北/东偏移 >20m)");
        check(v->missionCurrent() == 255 && !v->missionActive(),
              "任务结束: 当前航点复位 (MISSION_CURRENT=255)");
        // 参数管理 (阶段3)
        check(v->paramCount() == 8, "参数列表读取完整 (8 个)");
        check(gotParamList, "收到参数列表 (含 MPC_XY_CRUISE)");
        check(std::abs(paramCruiseValue - 8.5f) < 1e-3f,
              "PARAM_SET 修改被飞控确认回传 (MPC_XY_CRUISE=8.5)");
        check(v->paramType(QStringLiteral("MPC_XY_CRUISE")) == mav::PARAM_TYPE_REAL32,
              "参数类型正确 (FLOAT)");
        std::printf("\n%s (%d 项)\n", g_fail ? "存在失败项" : "全部通过", g_fail);
        app.exit(g_fail ? 1 : 0);
    });

    return app.exec();
}
