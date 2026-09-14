// ============================================================================
// SkyGCS  飞行日志 记录/回放 往返测试
//   验证: CSV 落盘格式、采样点数、回放后 VehicleState 状态还原
// ============================================================================
#include <QCoreApplication>
#include <QFile>
#include <QTimer>

#include <cmath>
#include <cstdio>

#include "core/flightlog.h"
#include "core/vehicle.h"
#include "mavlink/mavlink_types.h"

using namespace skygcs;

namespace {
int failures = 0;
int checks = 0;

void check(bool ok, const char* what)
{
    ++checks;
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        ++failures;
}
} // namespace

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    VehicleState v;
    const QString tmpFile = QStringLiteral("%1/skygcs_test_flightlog.csv")
                                .arg(QCoreApplication::applicationDirPath());
    QFile::remove(tmpFile);

    FlightLogRecorder recorder(&v);
    FlightLogPlayer player(&v);

    check(recorder.start(tmpFile), "记录器打开 CSV 文件");

    // 阶段 A: 未解锁/手动/家点附近
    v.updateHeartbeat(1, 1, mav::TYPE_QUADROTOR, mav::AUTOPILOT_PX4,
                      mav::MODE_FLAG_CUSTOM_MODE_ENABLED | mav::MODE_FLAG_MANUAL_INPUT_ENABLED,
                      mav::px4CustomModeEncode(mav::PX4_MODE_MANUAL, 0), mav::STATE_ACTIVE);
    v.updateGlobalPosition(24.51, 117.65, 30.0, 0.0, 0, 0, 0, 90);

    QTimer::singleShot(800, [&]() {
        // 阶段 B: 解锁/定高/起飞后位置
        v.updateHeartbeat(1, 1, mav::TYPE_QUADROTOR, mav::AUTOPILOT_PX4,
                          mav::MODE_FLAG_CUSTOM_MODE_ENABLED
                              | mav::MODE_FLAG_SAFETY_ARMED | mav::MODE_FLAG_AUTO_ENABLED,
                          mav::px4CustomModeEncode(mav::PX4_MODE_AUTO, mav::PX4_AUTO_TAKEOFF),
                          mav::STATE_ACTIVE);
        v.updateGlobalPosition(24.511, 117.652, 38.0, 8.0, 0, 0, 0, 135);
        v.updateVfrHud(4.5, 4.5, 2.0, 135, 60);
        v.updateBattery(16.5, 12.0, 87, 300);
        v.updateMissionCurrent(0, true);
    });

    QTimer::singleShot(1200, [&]() {
        recorder.stop();
        check(recorder.sampleCount() >= 2, "至少采样 2 个点 (500ms 周期)");

        QFile f(tmpFile);
        check(f.open(QIODevice::ReadOnly), "CSV 文件可读");
        if (f.isOpen()) {
            const QByteArray head = f.readLine();
            check(head.startsWith("t_ms,armed,mode_main,"), "表头字段正确");
            int lines = 0;
            while (!f.atEnd()) {
                f.readLine();
                ++lines;
            }
            check(lines == recorder.sampleCount(), "数据行数 == 采样点数");
            f.close();
        }

        // 回放
        check(player.load(tmpFile), "回放器加载 CSV");
        check(player.rowCount() == recorder.sampleCount(), "回放行数一致");
        player.play();
    });

    QTimer::singleShot(2200, [&]() {
        // 回放应已把最后一行(阶段 B)写入 VehicleState
        check(v.armed(), "回放后 armed=true (阶段B)");
        check(std::abs(v.lat() - 24.511) < 1e-6, "回放后纬度还原 24.511");
        check(std::abs(v.relAlt() - 8.0) < 1e-6, "回放后相对高度还原 8.0m");
        check(v.missionCurrent() == 0, "回放后任务当前航点=0");
        check(v.missionActive(), "回放后任务激活");
        check(v.batteryRemaining() == 87, "回放后电量 87%");
        check(v.modeName().contains("Takeoff"), "回放后模式=自动-起飞");

        QFile::remove(tmpFile);
        std::printf(checks ? "\n" : "", 0);
        std::printf("%s (%d 项, %d 失败)\n", failures ? "存在失败" : "全部通过", checks, failures);
        app.exit(failures ? 1 : 0);
    });

    return app.exec();
}
