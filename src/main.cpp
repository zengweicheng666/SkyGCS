// ============================================================================
// SkyGCS 无人机地面站 — 程序入口
// 自研 MAVLink 协议栈 + QT Widgets UI + 内置 PX4 仿真器
// ============================================================================
#include <QApplication>

#include "ui/mainwindow.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    QApplication::setApplicationName(QStringLiteral("SkyGCS"));
    QApplication::setOrganizationName(QStringLiteral("SkyGCS"));

    skygcs::MainWindow w;
    w.show();
    return app.exec();
}
