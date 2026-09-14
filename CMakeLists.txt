cmake_minimum_required(VERSION 3.16)
project(SkyGCS VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# MSVC: 源文件统一 UTF-8 (默认按本地代码页 GBK 解析会导致中文注释破坏语法)
if(MSVC)
    add_compile_options(/utf-8)
endif()

find_package(Qt6 COMPONENTS Widgets Network SerialPort Charts QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 5.15 COMPONENTS Widgets Network SerialPort Charts REQUIRED)
endif()

add_executable(skygcs
    src/main.cpp
    src/mavlink/mavlink_codec.cpp
    src/core/vehicle.cpp
    src/core/flightlog.cpp
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
    src/ui/missionpanel.cpp
    src/ui/parameterpanel.cpp
    src/ui/flightlogpanel.cpp
    src/ui/telemetrychart.cpp
    src/ui/linkdialog.cpp
    src/ui/serialconsole.cpp
    src/ui/messageinspector.cpp
    src/ui/mainwindow.cpp
)

target_include_directories(skygcs PRIVATE src)
target_link_libraries(skygcs PRIVATE Qt::Widgets Qt::Network Qt::SerialPort Qt::Charts)

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

    # 2b) 飞行日志 记录/回放 往返测试
    add_executable(test_flightlog tests/test_flightlog.cpp
        src/mavlink/mavlink_codec.cpp
        src/core/vehicle.cpp
        src/core/flightlog.cpp)
    target_include_directories(test_flightlog PRIVATE src)
    target_link_libraries(test_flightlog PRIVATE Qt::Core)

    # 2c) 载荷协议自动化测试 (HDLC/Modbus/SLCAN 字节级比对)
    add_executable(test_payload tests/test_payload.cpp
        src/comm/linkinterface.h
        src/comm/serialframeprotocol.cpp
        src/comm/slcanprotocol.cpp)
    target_include_directories(test_payload PRIVATE src)
    target_link_libraries(test_payload PRIVATE Qt::Core)

    # 2d) 参数驱动仿真物理 端到端测试 (MPC_* 参数真正影响飞行行为)
    add_executable(test_param_physics tests/test_param_physics.cpp
        src/comm/linkinterface.h
        src/mavlink/mavlink_codec.cpp
        src/core/vehicle.cpp
        src/comm/udplink.cpp
        src/comm/mavlinkendpoint.cpp
        src/sim/flightsim.cpp)
    target_include_directories(test_param_physics PRIVATE src)
    target_link_libraries(test_param_physics PRIVATE Qt::Core Qt::Network)

    # 2e) 独立仿真服务 (无 GUI, 供 pymavlink 等第三方地面站互通验证)
    add_executable(sim_standalone tests/sim_standalone.cpp
        src/comm/linkinterface.h
        src/mavlink/mavlink_codec.cpp
        src/core/vehicle.cpp
        src/comm/udplink.cpp
        src/comm/mavlinkendpoint.cpp
        src/sim/flightsim.cpp)
    target_include_directories(sim_standalone PRIVATE src)
    target_link_libraries(sim_standalone PRIVATE Qt::Core Qt::Network)

    # 3) UI 离屏渲染截图 (验证/文档用)
    add_executable(shot_main tests/shot_main.cpp
        src/comm/linkinterface.h
        src/mavlink/mavlink_codec.cpp
        src/core/vehicle.cpp
        src/core/flightlog.cpp
        src/comm/udplink.cpp
        src/comm/seriallink.cpp
        src/comm/mavlinkendpoint.cpp
        src/comm/serialframeprotocol.cpp
        src/comm/slcanprotocol.cpp
        src/sim/flightsim.cpp
        src/ui/attitudeindicator.cpp
        src/ui/telemetrypanel.cpp
        src/ui/commandpanel.cpp
        src/ui/missionpanel.cpp
        src/ui/parameterpanel.cpp
        src/ui/flightlogpanel.cpp
        src/ui/telemetrychart.cpp
        src/ui/linkdialog.cpp
        src/ui/serialconsole.cpp
        src/ui/messageinspector.cpp
        src/ui/mainwindow.cpp)
    target_include_directories(shot_main PRIVATE src)
    target_link_libraries(shot_main PRIVATE Qt::Widgets Qt::Network Qt::SerialPort Qt::Charts)
endif()
