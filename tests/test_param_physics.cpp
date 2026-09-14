// ============================================================================
// SkyGCS  参数驱动仿真物理 端到端测试
// 验证: PARAM_SET 修改后真正影响飞控行为 ——
//   MPC_Z_VEL_MAX: 提高垂直速度上限 → 起飞爬升速度可突破默认 3.0 m/s
//   MPC_XY_CRUISE: 航点巡航速度被限制到设定值
//   RTL_RETURN_ALT: 返航高度爬升到设定值
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
static int g_checks = 0;
static void check(bool ok, const char* what)
{
    ++g_checks;
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
        std::printf("[FAIL] UDP 14550 绑定失败\n");
        return 1;
    }
    endpoint->addLink(link);

    double peakClimb = 0;       // 起飞爬升峰值 (m/s)
    double peakSpeed = 0;       // 航点巡航速度峰值 (m/s)
    double peakRtlAlt = 0;      // 返航段高度峰值 (m)
    int paramCount = 0;
    bool inMissionWindow = false;

    QObject::connect(endpoint->vehicle(), &VehicleState::stateChanged, [&]() {
        const VehicleState* v = endpoint->vehicle();
        paramCount = v->paramCount();
        if (v->relAlt() > 1.0 && v->armed()) {
            peakClimb = std::max(peakClimb, v->climb());
            peakRtlAlt = std::max(peakRtlAlt, v->relAlt());
            if (inMissionWindow)
                peakSpeed = std::max(peakSpeed, v->groundspeed());
        }
    });

    sim->setHome(24.51, 117.65, 30.0);
    if (!sim->startUdpServer(14550)) {
        std::printf("[FAIL] 仿真器启动失败\n");
        return 1;
    }

    QTimer::singleShot(2500, [&]() { endpoint->requestParamList(); });

    QTimer::singleShot(3500, [&]() {
        // 提高垂直速度上限: 5 m/s (默认 3)
        endpoint->setParam(QStringLiteral("MPC_Z_VEL_MAX"), 5.0f);
    });

    QTimer::singleShot(5000, [&]() {
        endpoint->armDisarm(true);
    });
    QTimer::singleShot(5500, [&]() {
        endpoint->takeoff(20.0f);    // 爬升 20m, 观察速度是否突破 3.0
    });

    QTimer::singleShot(12500, [&]() {
        // 限制巡航速度: 1 m/s
        endpoint->setParam(QStringLiteral("MPC_XY_CRUISE"), 1.0f);
    });

    QTimer::singleShot(14000, [&]() {
        QVector<MavlinkEndpoint::MissionItem> items;
        MavlinkEndpoint::MissionItem it;
        it.frame = mav::FRAME_GLOBAL_RELATIVE_ALT_INT;
        it.z = 8.0f;
        it.x = static_cast<int32_t>((24.51 + 25.0 / 111320.0) * 1e7);
        it.y = static_cast<int32_t>(117.65 * 1e7);
        items.append(it);
        endpoint->uploadMission(items);
    });
    QTimer::singleShot(15000, [&]() {
        inMissionWindow = true;
        endpoint->startMission();
    });

    QTimer::singleShot(22000, [&]() {
        inMissionWindow = false;
        // 设置返航高度 35m 并发返航
        endpoint->setParam(QStringLiteral("RTL_RETURN_ALT"), 35.0f);
    });
    QTimer::singleShot(23500, [&]() { endpoint->rtl(); });

    QTimer::singleShot(31000, [&]() {
        const VehicleState* v = endpoint->vehicle();
        check(paramCount >= 8, "参数列表读取完整 (>=8)");
        float zvel = 0, cruise = 0;
        check(v->paramValue(QStringLiteral("MPC_Z_VEL_MAX"), zvel),
              "MPC_Z_VEL_MAX 存在");
        v->paramValue(QStringLiteral("MPC_XY_CRUISE"), cruise);
        check(std::abs(zvel - 5.0f) < 1e-3f, "MPC_Z_VEL_MAX 已确认 = 5.0");
        check(std::abs(cruise - 1.0f) < 1e-3f, "MPC_XY_CRUISE 已确认 = 1.0");
        // 垂直速度上限 5 → 起飞爬升应超过默认 3.0 上限
        check(peakClimb > 3.2, "爬升峰值速度 > 3.2 m/s (MPC_Z_VEL_MAX=5 生效)");
        std::printf("  [数据] peakClimb=%.2f m/s\n", peakClimb);
        // 巡航 1.0 → 航点跟踪速度被限制
        check(peakSpeed <= 1.8, "航点巡航速度峰值 <= 1.8 m/s (MPC_XY_CRUISE=1 生效)");
        std::printf("  [数据] peakSpeed=%.2f m/s\n", peakSpeed);
        // 返航高度 35 → 返航段高度爬升超过 30m
        check(peakRtlAlt > 30.0, "返航高度 > 30 m (RTL_RETURN_ALT=35 生效)");
        std::printf("  [数据] peakRtlAlt=%.1f m, mode=%s\n",
                    peakRtlAlt, qPrintable(v->modeName()));
        std::printf("\n%s (%d/%d 项)\n", g_fail ? "存在失败" : "全部通过",
                    g_checks - g_fail, g_checks);
        app.exit(g_fail ? 1 : 0);
    });

    return app.exec();
}
