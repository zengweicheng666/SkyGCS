#pragma once
// ============================================================================
// SkyGCS  MAVLink 协议引擎 (对应 QGC 的 MAVLinkProtocol + Vehicle)
//   - 挂接 LinkInterface, 逐链路维护编解码器
//   - 遥测消息解码 → 更新 VehicleState
//   - 周期心跳 (1Hz)
//   - 指令下发 (COMMAND_LONG) + ACK 跟踪/超时重发
//   - 任意消息转发给检查器 (messageReceived)
// ============================================================================
#include <QObject>
#include <QTimer>
#include <QHash>

#include "../core/vehicle.h"
#include "../mavlink/mavlink_codec.h"
#include "../mavlink/mavlink_messages.h"
#include "linkinterface.h"

namespace skygcs {

class MavlinkEndpoint : public QObject {
    Q_OBJECT
public:
    explicit MavlinkEndpoint(QObject* parent = nullptr);

    // 链路管理 (支持同时挂多条: 数传 + 串口)
    void addLink(LinkInterface* link);
    void removeLink(LinkInterface* link);
    QList<LinkInterface*> links() const { return links_; }

    VehicleState* vehicle() { return &vehicle_; }
    const VehicleState* vehicle() const { return &vehicle_; }

    // ---- 指令 API (均返回是否成功入队) ----
    bool sendCommandLong(uint16_t command, float p1 = 0, float p2 = 0, float p3 = 0,
                         float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0);
    bool sendSetMode(uint32_t mainMode, uint32_t subMode = 0);   // PX4 模式
    bool armDisarm(bool arm);
    bool takeoff(float altM = 5.0f);
    bool land();
    bool rtl();
    bool requestHome();
    bool sendPing();

    // ---- 任务 (Mission) ----
    struct MissionItem {
        uint16_t command = mav::CMD_NAV_WAYPOINT;
        uint8_t  frame = mav::FRAME_GLOBAL_RELATIVE_ALT_INT;
        float    p1 = 0, p2 = 0, p3 = 0, p4 = 0;   // 参数 (通常 0)
        int32_t  x = 0;                            // 纬度 degE7
        int32_t  y = 0;                            // 经度 degE7
        float    z = 5.0f;                         // 高度 m (相对)
    };
    // 异步上传: COUNT → 收到 REQUEST_INT(seq) → 发 ITEM(seq) → ... → ACK
    bool uploadMission(const QVector<MissionItem>& items);
    bool startMission();        // CMD_MISSION_START
    bool abortMissionUpload();  // 取消上传流程

    // ---- 参数 (PARAM) ----
    bool requestParamList();                          // 请求全部参数
    bool readParam(const QString& name);              // 按名读取单个参数
    bool setParam(const QString& name, float value, uint8_t type = mav::PARAM_TYPE_REAL32);

    // 发送任意 MavMessage (用于仿真注入/扩展)
    bool sendMessage(const MavMessage& msg);

    // 注入模式: 仿真器直接送入解码/分发路径 (不经网络)
    void injectMessage(const MavMessage& msg) { handleMessage(msg); }

signals:
    void vehicleOnline(bool online);
    void messageReceived(const skygcs::MavMessage& msg);      // 全部解码消息
    void commandAck(uint16_t command, uint8_t result, int progress, int resultParam2);
    void statustext(int severity, const QString& text);
    void logMessage(const QString& text);                     // 引擎日志

private slots:
    void onLinkBytes(const QByteArray& data);
    void onHeartbeatTimer();
    void onAckTimer();

private:
    void handleMessage(const MavMessage& msg);
    void sendFrame(LinkInterface* link, const MavMessage& msg);

    // 根据消息 ID 尝试解码并更新车辆状态
    void handleHeartbeat(const MavMessage& msg);
    void handleAttitude(const MavMessage& msg);
    void handleGlobalPosition(const MavMessage& msg);
    void handleGpsRaw(const MavMessage& msg);
    void handleVfrHud(const MavMessage& msg);
    void handleBattery(const MavMessage& msg);
    void handleSysStatus(const MavMessage& msg);
    void handleStatustext(const MavMessage& msg);
    void handleHome(const MavMessage& msg);
    void handleRadio(const MavMessage& msg);
    void handleCommandAck(const MavMessage& msg);
    void handlePing(const MavMessage& msg);
    void handleMissionRequestInt(const MavMessage& msg);
    void handleMissionAck(const MavMessage& msg);
    void handleMissionCurrent(const MavMessage& msg);
    void handleMissionItemReached(const MavMessage& msg);
    void handleParamValue(const MavMessage& msg);

    QList<LinkInterface*> links_;
    QHash<LinkInterface*, MavlinkCodec*> codecs_;

    VehicleState vehicle_;
    QTimer heartbeatTimer_;
    QTimer ackTimer_;

    // 指令 ACK 跟踪
    struct PendingCommand {
        uint16_t command = 0;
        CommandLongMsg params{};       // 保留原始参数用于重发
        int retries = 0;
        qint64 sentAtMs = 0;
    };
    PendingCommand pending_;
    uint8_t targetSysid_ = 1;
    uint8_t targetCompid_ = 1;

    // 任务上传状态机
    struct MissionUpload {
        QVector<MissionItem> items;
        int nextSeq = 0;
        bool active = false;
        qint64 startedAtMs = 0;
    };
    MissionUpload missionUpload_;

    LinkInterface* lastLink_ = nullptr;    // 最近收到数据的链路 (用于回复)
};

} // namespace skygcs
