// ============================================================================
// SkyGCS  独立仿真服务 (无 GUI)
// 以 UDP 14550 提供内置 PX4 仿真遥测, 供第三方地面站/脚本 (如 pymavlink) 互通验证
// 运行: build/sim_standalone.exe [秒数=45]
// ============================================================================
#include <QCoreApplication>
#include <QTimer>
#include <cstdio>

#include "../src/sim/flightsim.h"

using namespace skygcs;

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    FlightSim sim;
    sim.setHome(24.51, 117.65, 30.0);
    if (!sim.startUdpServer(14550)) {
        std::printf("FAIL: 仿真 UDP 服务启动失败\n");
        return 1;
    }
    std::printf("SkyGCS 内置仿真 UDP 服务运行中 (遥测 -> 127.0.0.1:14550)\n");
    std::fflush(stdout);

    const int seconds = (argc > 1) ? std::atoi(argv[1]) : 45;
    QTimer::singleShot(seconds * 1000, &app, &QCoreApplication::quit);
    return app.exec();
}
