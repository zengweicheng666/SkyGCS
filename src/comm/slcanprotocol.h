#pragma once
// ============================================================================
// SkyGCS  SLCAN (Serial CAN) 协议
// 通过串口桥接 CAN 总线 (适配 CANable / USB-CAN 模块, 常用于 DroneCAN/CAN 载荷)
//
// 常用指令 (Lawicel 格式):
//   tIIIC           发送标准帧 (11bit ID, 1-8 字节)
//   TIIIIIIIIC      发送扩展帧 (29bit ID)
//   rIII/RIIIIIIII  发送远程帧
//   z / Z           打开/关闭 CAN
//   V               版本
// ============================================================================
#include <QObject>
#include <QByteArray>

#include "linkinterface.h"

namespace skygcs {

struct CanFrame {
    uint32_t id = 0;
    bool     extended = false;
    bool     rtr = false;
    uint8_t  dlc = 0;
    uint8_t  data[8] = {0};

    // 人类可读: [ext]ID  dlc  data hex
    QString toString() const;
};

class SlcanProtocol : public QObject {
    Q_OBJECT
public:
    explicit SlcanProtocol(QObject* parent = nullptr);

    void attach(LinkInterface* link);
    void detach();

    // 打开/关闭 CAN 通道 (发送 z/Z 指令)
    bool sendOpenCan();
    bool sendCloseCan();
    bool sendVersion();

    // 发送 CAN 帧
    bool sendFrame(const CanFrame& frame);

signals:
    void frameReceived(const skygcs::CanFrame& frame);
    void logReceived(const QString& text);

private slots:
    void onBytes(const QByteArray& data);

private:
    void parseLine(const QByteArray& line);

    LinkInterface* link_ = nullptr;
    QByteArray lineBuf_;
};

} // namespace skygcs
