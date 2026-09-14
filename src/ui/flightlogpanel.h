#pragma once
// ============================================================================
// SkyGCS  飞行日志面板
//   记录: 启动/停止遥测 CSV 落盘 (默认 logs/ 目录)
//   回放: 打开 CSV → 播放/暂停/停止/倍速/进度, 驱动全 UI 联动
// ============================================================================
#include <QWidget>

class QLabel;
class QLineEdit;
class QPushButton;
class QProgressBar;
class QSlider;

namespace skygcs {

class FlightLogRecorder;
class FlightLogPlayer;
class MavlinkEndpoint;

class FlightLogPanel : public QWidget {
    Q_OBJECT
public:
    explicit FlightLogPanel(MavlinkEndpoint* endpoint, QWidget* parent = nullptr);

public slots:
    void startRecording();
    void stopRecording();
    void openAndPlay();
    void playPause();
    void stopPlay();
    void onProgress(int row, int total);
    void onLog(const QString& text);

private:
    void setRecControls(bool recording);
    void setPlayControls(bool loaded, bool playing);

    MavlinkEndpoint* endpoint_ = nullptr;
    FlightLogRecorder* recorder_ = nullptr;
    FlightLogPlayer* player_ = nullptr;

    QLineEdit* recPath_ = nullptr;
    QPushButton* btnRecStart_ = nullptr;
    QPushButton* btnRecStop_ = nullptr;
    QLabel* recStatus_ = nullptr;

    QLabel* playFile_ = nullptr;
    QPushButton* btnOpen_ = nullptr;
    QPushButton* btnPlay_ = nullptr;
    QPushButton* btnStop_ = nullptr;
    QSlider* speedSlider_ = nullptr;
    QLabel* speedLabel_ = nullptr;
    QProgressBar* progress_ = nullptr;
};

} // namespace skygcs
