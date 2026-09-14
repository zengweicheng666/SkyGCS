// ============================================================================
// SkyGCS  载荷协议自动化测试 (字节级)
//   HDLC:   CRC16-CCITT 已知向量 + 帧封装/转义/解析往返 + 错误帧丢弃
//   Modbus: CRC16-Modbus 已知向量 + 标准请求帧解析 + 发送帧校验
//   SLCAN:  Lawicel 标准/扩展帧 收发往返 (11bit/29bit ID)
// ============================================================================
#include <QCoreApplication>
#include <QByteArray>
#include <QEventLoop>
#include <QTimer>
#include <cstdio>

#include "../src/comm/linkinterface.h"
#include "../src/comm/serialframeprotocol.h"
#include "../src/comm/slcanprotocol.h"

using namespace skygcs;

// 内存假链路: sendBytes 捕获, bytesReceived 可注入
class FakeLink : public LinkInterface {
public:
    using LinkInterface::LinkInterface;
    bool open() override { return true; }
    void close() override {}
    bool isOpen() const override { return true; }
    QString name() const override { return QStringLiteral("fake"); }
    QString detail() const override { return QStringLiteral("test"); }
    void sendBytes(const QByteArray& data) override { tx_.append(data); }
    QByteArray tx_;
};

static int g_fail = 0;
static int g_checks = 0;
static void check(bool ok, const char* what)
{
    ++g_checks;
    std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok)
        ++g_fail;
}

// 十六进制字面量转字节
static QByteArray hex(const char* s)
{
    QByteArray out;
    while (*s && *(s + 1)) {
        out.append(static_cast<char>((QByteArray::fromHex(QByteArray(s, 2)).at(0))));
        s += 2;
    }
    return out;
}

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    // ==================== HDLC ====================
    {
        // CRC16-CCITT 已知向量: "123456789" → 0x29B1
        check(SerialFrameProtocol::crc16Ccitt(QByteArrayLiteral("123456789")) == 0x29B1,
              "CRC16-CCITT 已知向量 123456789 -> 0x29B1");

        FakeLink link;
        SerialFrameProtocol proto;
        proto.attach(&link);

        // 发送帧: 含转义字节的 payload (0x7E, 0x7D) 验证转义正确
        QByteArray payload;
        payload.append(static_cast<char>(0x01));
        payload.append(static_cast<char>(0x7E));
        payload.append(static_cast<char>(0x7D));
        payload.append(static_cast<char>(0x00));
        check(proto.sendHdlcFrame(0x05, 0x10, payload), "HDLC 发送含转义字节帧");

        // 捕获字节流必须以 0x7E 首尾; 帧体无裸 0x7E, 0x7D 后必须跟 0x5E/0x5D
        const QByteArray& tx = link.tx_;
        check(tx.size() > 4 && static_cast<quint8>(tx.at(0)) == 0x7E
                  && static_cast<quint8>(tx.at(tx.size() - 1)) == 0x7E,
              "HDLC 帧以标志字节包裹");
        bool escapingOk = true;
        for (int i = 1; i < tx.size() - 1; ++i) {
            const quint8 b = static_cast<quint8>(tx.at(i));
            if (b == 0x7E) { escapingOk = false; break; }          // 裸标志
            if (b == 0x7D) {                                       // 转义前缀
                if (i + 1 >= tx.size() - 1) { escapingOk = false; break; }
                const quint8 n = static_cast<quint8>(tx.at(i + 1));
                if (n != 0x5E && n != 0x5D) { escapingOk = false; break; }
                ++i;
            }
        }
        check(escapingOk, "HDLC 帧体转义正确 (无裸标志/转义对合法)");

        // 注入该帧回环解析
        int gotAddr = -1, gotFunc = -1;
        QByteArray gotPayload;
        QObject::connect(&proto, &SerialFrameProtocol::hdlcFrameReceived,
                         [&](uint8_t a, uint8_t f, const QByteArray& p) {
            gotAddr = a; gotFunc = f; gotPayload = p;
        });
        emit link.bytesReceived(tx);
        check(gotAddr == 0x05 && gotFunc == 0x10 && gotPayload == payload,
              "HDLC 回环解析还原 addr/func/payload (含转义)");

        // 已知字节流注入: 手工构造帧 7E 01 02 03 00 AA 55 CRC 7E
        //   body = 01 02 03 00 AA 55 (addr=01 func=02 len=03 payload=00AA55)
        QByteArray body = hex("01020300AA55");
        const quint16 crc = SerialFrameProtocol::crc16Ccitt(body);
        QByteArray frame;
        frame.append(static_cast<char>(0x7E));
        frame.append(body);
        frame.append(static_cast<char>(crc & 0xFF));
        frame.append(static_cast<char>((crc >> 8) & 0xFF));
        frame.append(static_cast<char>(0x7E));
        link.tx_.clear();
        gotAddr = -1; gotFunc = -1; gotPayload.clear();
        emit link.bytesReceived(frame);
        check(gotAddr == 0x01 && gotFunc == 0x02 && gotPayload == hex("00AA55"),
              "HDLC 已知字节流解析 (addr=0x01 func=0x02 payload=00AA55)");

        // 错误帧 (CRC 损坏) 应被丢弃, 不触发信号
        QByteArray bad = frame;
        bad[bad.size() - 2] = static_cast<char>(bad.at(bad.size() - 2) ^ 0xFF);
        gotAddr = -1;
        emit link.bytesReceived(bad);
        check(gotAddr == -1, "HDLC CRC 错误帧被丢弃");
    }

    // ==================== Modbus ====================
    {
        // CRC16-Modbus 已知向量: "123456789" → 0x4B37
        check(SerialFrameProtocol::crc16Modbus(QByteArrayLiteral("123456789")) == 0x4B37,
              "CRC16-Modbus 已知向量 123456789 -> 0x4B37");

        FakeLink link;
        SerialFrameProtocol proto;
        proto.attach(&link);

        int gotAddr = -1, gotFunc = -1;
        QByteArray gotData;
        QObject::connect(&proto, &SerialFrameProtocol::modbusFrameReceived,
                         [&](uint8_t a, uint8_t f, const QByteArray& d) {
            gotAddr = a; gotFunc = f; gotData = d;
        });

        // 标准 Modbus RTU 读保持寄存器请求: 01 03 00 00 00 0A C5 CD
        // (30ms 静默定时器切帧, 需等待事件循环)
        emit link.bytesReceived(hex("01030000000AC5CD"));
        {
            QEventLoop loop;
            QTimer::singleShot(80, &loop, &QEventLoop::quit);
            loop.exec();
        }
        check(gotAddr == 0x01 && gotFunc == 0x03 && gotData == hex("0000000A"),
              "Modbus RTU 标准帧解析 (读 10 个保持寄存器)");

        // 发送帧 CRC 正确性: 重新解析自身
        link.tx_.clear();
        check(proto.sendModbus(0x02, 0x06, hex("00010005")), "Modbus 发送写单寄存器");
        check(!link.tx_.isEmpty(), "Modbus 发送有数据");
        if (!link.tx_.isEmpty()) {
            const QByteArray body = link.tx_.left(link.tx_.size() - 2);
            const quint16 crcTx = static_cast<quint8>(link.tx_.at(link.tx_.size() - 2))
                                | (static_cast<quint8>(link.tx_.at(link.tx_.size() - 1)) << 8);
            check(SerialFrameProtocol::crc16Modbus(body) == crcTx,
                  "Modbus 发送帧 CRC 自校验一致");
        }

        // 无静默切帧: 粘包 (两帧连续) 30ms 静默后应解析出 2 帧
        int rxCount = 0;
        QObject::connect(&proto, &SerialFrameProtocol::modbusFrameReceived,
                         [&](uint8_t, uint8_t, const QByteArray&) { ++rxCount; });
        emit link.bytesReceived(hex("01030000000AC5CD01030000000AC5CD"));
        {
            QEventLoop loop;
            QTimer::singleShot(80, &loop, &QEventLoop::quit);
            loop.exec();
        }
        check(rxCount == 2, "Modbus 粘包 30ms 静默切帧解析 2 帧");
    }

    // ==================== SLCAN ====================
    {
        FakeLink link;
        SlcanProtocol proto;
        proto.attach(&link);

        // 标准帧发送: id=0x123 dlc=3 data=DE AD BE → "t1233deadbe\r" (Qt arg 十六进制小写)
        CanFrame stdFrame;
        stdFrame.id = 0x123;
        stdFrame.dlc = 3;
        stdFrame.data[0] = 0xDE; stdFrame.data[1] = 0xAD; stdFrame.data[2] = 0xBE;
        check(proto.sendFrame(stdFrame), "SLCAN 发送标准帧");
        check(link.tx_ == QByteArray("t1233deadbe\r"), "SLCAN 标准帧 ASCII 正确");

        // 扩展帧发送: id=0x1F234567 dlc=1 data=11 → "T1f234567111\r"
        CanFrame extFrame;
        extFrame.extended = true;
        extFrame.id = 0x1F234567;
        extFrame.dlc = 1;
        extFrame.data[0] = 0x11;
        link.tx_.clear();
        check(proto.sendFrame(extFrame), "SLCAN 发送扩展帧");
        check(link.tx_ == QByteArray("T1f234567111\r"), "SLCAN 扩展帧 ASCII 正确");

        // 接收回环: 标准帧 (t + 123 + 8 + 8 字节数据 = 21 字符)
        CanFrame rx;
        bool got = false;
        QObject::connect(&proto, &SlcanProtocol::frameReceived,
                         [&](const CanFrame& f) { rx = f; got = true; });
        emit link.bytesReceived(QByteArray("t1238DEADBEEF01020304\r"));
        check(got && rx.id == 0x123 && rx.dlc == 8 && !rx.extended,
              "SLCAN 标准帧接收解析");
        check(rx.data[0] == 0xDE && rx.data[7] == 0x04, "SLCAN 标准帧数据正确");

        // 接收回环: 扩展帧
        got = false;
        emit link.bytesReceived(QByteArray("T1F23456780102030405060708\r"));
        check(got && rx.id == 0x1F234567 && rx.extended && rx.dlc == 8,
              "SLCAN 扩展帧接收解析");
        check(rx.data[7] == 0x08, "SLCAN 扩展帧数据正确");

        // 非法行: 不应触发 frameReceived
        got = false;
        emit link.bytesReceived(QByteArray("x123\r"));
        check(!got, "SLCAN 非法行被忽略");
    }

    std::printf("\n%s (%d 项, %d 失败)\n", g_fail ? "存在失败" : "全部通过",
                g_checks, g_fail);
    return g_fail ? 1 : 0;
}
