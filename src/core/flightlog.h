#pragma once
// ============================================================================
// SkyGCS  飞行日志
//   FlightLogRecorder: 遥测采样 → CSV 落盘 (500ms, 订阅 VehicleState)
//   FlightLogPlayer  : 读取 CSV → 逐行回放, 驱动 VehicleState → UI 联动
// ============================================================================
#include <QFile>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QTimer>

namespace skygcs {

class VehicleState;

// ---------------------------------------------------------------------------
// CSV 列定义 (表头即字段名, 回放按列序解析)
// t_ms, armed, mode_main, mode_sub, lat, lon, alt_msl, rel_alt,
// climb, groundspeed, heading, battery, mission_cur, mission_reached
// ---------------------------------------------------------------------------
class FlightLogRecorder : public QObject {
    Q_OBJECT
public:
    explicit FlightLogRecorder(VehicleState* vehicle, QObject* parent = nullptr);

    bool start(const QString& filePath);
    void stop();
    bool recording() const { return recording_; }
    QString filePath() const { return filePath_; }
    int sampleCount() const { return samples_; }

signals:
    void logMessage(const QString& text);
    void sampleWritten(int count);

private slots:
    void onSample();

private:
    void writeHeader();

    VehicleState* vehicle_ = nullptr;
    QFile file_;
    QTextStream out_;
    QTimer timer_;
    bool recording_ = false;
    QString filePath_;
    int samples_ = 0;
};

// ---------------------------------------------------------------------------
// CSV 回放器
// ---------------------------------------------------------------------------
class FlightLogPlayer : public QObject {
    Q_OBJECT
public:
    explicit FlightLogPlayer(VehicleState* vehicle, QObject* parent = nullptr);

    bool load(const QString& filePath);
    void play();
    void pause();
    void stop();
    void setSpeed(double speed);          // 倍速 (0.5~10)
    bool jumpTo(int row);

    bool loaded() const { return !rows_.isEmpty(); }
    bool playing() const { return playing_; }
    int rowCount() const { return rows_.size(); }
    int currentRow() const { return row_; }
    double speed() const { return speed_; }

signals:
    void logMessage(const QString& text);
    void progressChanged(int row, int total);
    void finished();

private slots:
    void onStep();

private:
    void applyRow(int i);

    VehicleState* vehicle_ = nullptr;
    QStringList rows_;           // 数据行 (CSV 已按逗号切分存储)
    int row_ = -1;
    bool playing_ = false;
    double speed_ = 1.0;
    QTimer timer_;
};

} // namespace skygcs
