cmake_minimum_required(VERSION 3.16)
project(SkyGCS VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 COMPONENTS Widgets Network SerialPort QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 5.15 COMPONENTS Widgets Network SerialPort REQUIRED)
endif()

add_executable(skygcs
    src/main.cpp
    src/mavlink/mavlink_codec.cpp
    src/core/vehicle.cpp
    src/comm/linkinterface.h
    src/comm/seriallink.cpp
    src/comm/udplink.cpp
    src/comm/mavlinkendpoint.cpp
    src/comm/serialframeprotocol.cpp
    src/comm/slcanprotocol.cpp
    src/sim/flightsim.cpp
    src/ui/attitudeindicator.cpp
    src/ui/telemetrypanel.cpp
    src/ui/commandpanel.cpp
    src/ui/linkdialog.cpp
    src/ui/serialconsole.cpp
    src/ui/messageinspector.cpp
    src/ui/mainwindow.cpp
)

target_include_directories(skygcs PRIVATE src)
target_link_libraries(skygcs PRIVATE Qt::Widgets Qt::Network Qt::SerialPort)

# ---- 测试 ----
option(SKYGCS_BUILD_TESTS "构建协议/集成测试" ON)
if(SKYGCS_BUILD_TESTS)
    # 1) MAVLink 编解码交叉验证 (无 Qt, 纯 g++ 可编译)
    add_executable(test_codec tests/test_codec.cpp src/mavlink/mavlink_codec.cpp)
    target_include_directories(test_codec PRIVATE .)

    # 2) 端到端集成 (仿真器↔协议引擎, 需 Qt 事件循环)
    add_executable(test_integration tests/test_integration.cpp
        src/comm/linkinterface.h
        src/mavlink/mavlink_codec.cpp
        src/core/vehicle.cpp
        src/comm/udplink.cpp
        src/comm/mavlinkendpoint.cpp
        src/sim/flightsim.cpp)
    target_include_directories(test_integration PRIVATE src)
    target_link_libraries(test_integration PRIVATE Qt::Core Qt::Network)

    # 3) UI 离屏渲染截图 (验证/文档用)
    add_executable(shot_main tests/shot_main.cpp
        src/comm/linkinterface.h
        src/mavlink/mavlink_codec.cpp
        src/core/vehicle.cpp
        src/comm/udplink.cpp
        src/comm/seriallink.cpp
        src/comm/mavlinkendpoint.cpp
        src/comm/serialframeprotocol.cpp
        src/comm/slcanprotocol.cpp
        src/sim/flightsim.cpp
        src/ui/attitudeindicator.cpp
        src/ui/telemetrypanel.cpp
        src/ui/commandpanel.cpp
        src/ui/linkdialog.cpp
        src/ui/serialconsole.cpp
        src/ui/messageinspector.cpp
        src/ui/mainwindow.cpp)
    target_include_directories(shot_main PRIVATE src)
    target_link_libraries(shot_main PRIVATE Qt::Widgets Qt::Network Qt::SerialPort)
endif()
