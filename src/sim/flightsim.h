#pragma once
// ============================================================================
// SkyGCS  内置飞控仿真器 (模拟 PX4 四旋翼)
//
// 两种运行模式:
//  1) 注入模式 (inject): 直接向 MavlinkEndpoint 注入遥测消息, 无需网络
//  2) UDP 服务模式 (udpServer): 监听命令/心跳, 向地面站 (默认 127.0.0.1:14550)
//     发送遥测 —— 可对接本地面站或 QGroundControl, 验证协议互通
//
// 遥测: HEARTBEAT 1Hz / ATTITUDE 20Hz / GLOBAL_POSITION_INT 5Hz /
//       GPS_RAW_INT 2Hz / VFR_HUD 5Hz / SYS_STATUS 1Hz / BATTERY_STATUS 1Hz /
//       HOME_POSITION 1次 / STATUSTEXT 事件
// 指令: COMMAND_LONG(ARM/TAKEOFF/LAND/RTL) + ACK, SET_MODE, PING
// ============================================================================
#include <QObject>
#include <QTimer>
#include <QUdpSocket>
#include <vector>

#include "../mavlink/mavlink_codec.h"
#include "../mavlink/mavlink_messages.h"

namespace skygcs {

class FlightSim : public QObject {
    Q_OBJECT
public:
    explicit FlightSim(QObject* parent = nullptr);

    void setHome(double lat, double lon, double altMsl = 30.0);

    // 注入模式: 仿真消息直接通过 messageReady 发出
    bool startInject();
    // UDP 服务模式: 开始服务 (gcsPort 为地面站监听端口)
    bool startUdpServer(quint16 gcsPort = 14550);
    void stop();
    bool running() const { return running_; }
    QString mode() const { return mode_; }

signals:
    void messageReady(const skygcs::MavMessage& msg);   // 注入模式
    void logMessage(const QString& text);

private slots:
    void onTick();
    void onUdpReady();

private:
    void sendMessage(const MavMessage& msg);
    void handleCommand(const MavMessage& msg);
    void processCommandLong(const CommandLongMsg& c);
    void setMode(uint32_t mainMode, uint32_t subMode);
    QString modeName() const;
    void emitStatustext(int severity, const QString& text);
    void encodeAndSend(uint32_t msgid, const void* structPtr);

    // 物理状态
    double posN_[3] = {0, 0, 0};     // NED m
    double velN_[3] = {0, 0, 0};     // m/s
    double att_[3] = {0, 0, 0};      // roll/pitch/yaw rad
    double rates_[3] = {0, 0, 0};
    double homeLat_ = 24.51, homeLon_ = 117.65, homeAltMsl_ = 30.0;

    // 任务状态
    bool armed_ = false;
    uint32_t mainMode_ = 1;          // PX4_MODE_MANUAL
    uint32_t subMode_ = 0;
    double targetAlt_ = 5.0;
    double battery_ = 100.0;
    qint64 startMs_ = 0;

    // ---- 任务 (Mission) ----
    std::vector<MissionItemIntMsg> missionItems_;   // 已上传航点
    int missionCount_ = -1;                         // 期望航点数
    bool missionReady_ = false;                     // 上传完成 (ACK 已发)
    bool missionActive_ = false;                    // 正在执行
    int missionIdx_ = 0;                            // 当前航点
    bool missionReachedSent_ = false;

    // 节流发送计数
    int tick_ = 0;

    bool running_ = false;
    QString mode_;
    QTimer timer_;
    QUdpSocket udp_;
    MavlinkCodec codec_;
    QHostAddress gcsAddr_ = QHostAddress("127.0.0.1");
    quint16 gcsPort_ = 14550;
};

} // namespace skygcs
