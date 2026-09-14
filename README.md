# SkyGCS —— 无人机地面站实战项目（QT/C++ · MAVLink · PX4 · RS485/RS422/CAN）

> 结合 JD 技术要求（QT/C++ 桌面应用、无人机地面站开发、与 PX4 飞控通信（MAVLink）、RS485/RS422/CAN 串口通信）打造的**从协议栈到地面站 UI 的完整可运行工程**，含自研 MAVLink 编解码、内置 PX4 仿真器、三类串口载荷协议，全部通过自动化交叉验证。

---

## 一、调研结论：无人机飞控通信主流架构与仿真工具

### 1.1 地面站 ↔ 飞控通信协议：MAVLink 是事实标准

| 维度 | 结论 |
|---|---|
| 协议 | **MAVLink**（Micro Air Vehicle Link）：地面站 ↔ 飞控间的主流开放协议，QGroundControl / MAVSDK / PX4 / ArduPilot 全生态通用 |
| 帧格式 | v1（`0xFE` 头，6 字节头 + 载荷 + CRC）与 **v2**（`0xFD` 头，10 字节头，支持 24bit msgid、扩展字段、签名） |
| 校验 | **CRC-16/MCRF4XX**，逐字节累加（`len` 起、不含 STX），每条消息叠加官方 `crc_extra`（由消息名 + 各字段 C 类型/名称/数组长度字符串累加再折叠） |
| 字段序 | 部署生态按**类型长度降序**排列（`mavparse sort_fields=True`），与 XML 声明序不同 —— 这是手写编解码最容易踩的坑 |
| 消息 | HEARTBEAT(1Hz)、ATTITUDE(20Hz+)、GLOBAL_POSITION_INT、GPS_RAW_INT、VFR_HUD、SYS_STATUS、BATTERY_STATUS、COMMAND_LONG/ACK、SET_MODE、STATUSTEXT 等 |

### 1.2 飞控内部架构（PX4 参考）

- PX4 内部采用 **uORB 微内核发布/订阅总线**：传感器、估计器、控制器、导航模块解耦通信；MAVLink 只是其中一个"外部通信模块"，把 uORB 话题编码成 MAVLink 消息送出、把地面站指令解码回写 uORB。
- 地面站侧，QGC 的分层是 **LinkInterface（串口/UDP/TCP）→ MAVLinkProtocol（帧解析/校验）→ Vehicle（状态聚合）→ UI**。本项目直接借鉴该分层。

### 1.3 仿真体系与工具

| 层次 | 说明 | 主流工具 |
|---|---|---|
| **SITL**（软件在环） | 跑**真实飞控代码**，输入由仿真环境提供 | PX4 SITL + **Gazebo**、jMAVSim、AirSim、SIH；遥测默认 **UDP 14550** |
| **HITL**（硬件在环） | 真实飞控板 + 仿真传感器/执行机构 | X-Plane、Gazebo |
| 简单仿真 | 无需真实飞控 | 本项目的内置仿真器（模拟 PX4 心跳/遥测/指令 ACK/任务执行） |

> 对本项目：内置仿真器模拟"PX4 行为 + UDP 遥测"，因此**零硬件、零 PX4 安装**即可演示完整地面站闭环（含航点任务）；真实 PX4 SITL 只需把链路指向 `127.0.0.1:14550`。

---

## 二、JD 技术点 → 本项目落地映射

| JD 要求 | 本项目实现 |
|---|---|
| QT/C++ 桌面应用 | Qt 6 Widgets（兼容 Qt 5.15），CMake 工程，深色专业风格 |
| 无人机地面站开发 | 遥测监控（人工地平仪+仪表条+**实时趋势曲线**）、飞行指令、**任务规划（航点上传/执行/进度）**、**参数管理（PARAM 读写）**、**飞行日志（CSV 落盘+回放）**、消息检查器、系统日志 |
| 与 PX4 飞控通信（MAVLink） | **自研 MAVLink v1/v2 编解码**（不依赖第三方库，30 条消息 CRC 与官方全对齐） |
| RS485/RS422/CAN 串口通信 | ① 串口链路（QSerialPort，含 RS485/422 半双工说明）② HDLC 风格载荷帧协议 ③ Modbus RTU（CRC16-Modbus + 静默切帧）④ **SLCAN（CAN 桥）** 协议 |

---

## 三、工程架构

```
SkyGCS/
├─ CMakeLists.txt                     # 主工程 + 3 个测试目标
├─ src/
│  ├─ main.cpp                        # 入口
│  ├─ mavlink/                        # ★ 自研 MAVLink 协议栈（无 Qt 依赖，可独立测试）
│  │  ├─ mavlink_types.h              #   类型/常量/消息定义/px4CustomMode 编解码
│  │  ├─ mavlink_codec.h/.cpp         #   v1/v2 流式解析、CRC-16/MCRF4XX、打包/解包
│  │  ├─ mavlink_generated.h          #   由官方 c_library_v2 生成的消息表（32 条）
│  │  ├─ mavlink_messages.h           #   常用消息结构体 + 便捷 pack/unpack
│  │  └─ mavlink_names.h              #   指令/结果/参数类型名称（UI 层）
│  ├─ core/vehicle.h/.cpp             # VehicleState 状态聚合（信号驱动 UI）+ 参数表
│  ├─ core/flightlog.h/.cpp           # 飞行日志：遥测 CSV 落盘 + 回放器
│  ├─ comm/                           # 链路与协议引擎（对应 QGC LinkInterface/MAVLinkProtocol）
│  │  ├─ linkinterface.h              #   抽象链路
│  │  ├─ seriallink.h/.cpp            #   串口（RS232/RS485/RS422）
│  │  ├─ udplink.h/.cpp               #   UDP（对接 SITL/真机，QGC 式"回复最近对端"）
│  │  ├─ mavlinkendpoint.h/.cpp       #   协议引擎：1Hz 心跳、指令+ACK 跟踪重发、遥测/任务/参数分发
│  │  ├─ serialframeprotocol.h/.cpp   #   HDLC 帧协议 + Modbus RTU
│  │  └─ slcanprotocol.h/.cpp         #   SLCAN（CAN 桥）
│  ├─ sim/flightsim.h/.cpp            # ★ 内置 PX4 四旋翼仿真（20Hz 物理 + 注入/UDP 双模式 + 航点任务 + 参数表）
│  └─ ui/                             # 地面站界面
│     ├─ mainwindow.h/.cpp            #   主窗口/菜单/工具栏/日志/状态栏
│     ├─ attitudeindicator.h/.cpp     #   人工地平仪（自定义绘制）
│     ├─ telemetrypanel.h/.cpp        #   遥测面板
│     ├─ telemetrychart.h/.cpp        #   遥测趋势曲线（QtCharts：高度/垂直速度/电量）
│     ├─ commandpanel.h/.cpp          #   指令面板
│     ├─ missionpanel.h/.cpp          #   任务规划面板（航点编辑/上传/开始/进度）
│     ├─ parameterpanel.h/.cpp        #   参数管理面板（PARAM 协议读写/确认状态）
│     ├─ flightlogpanel.h/.cpp        #   飞行日志面板（记录控制 + 回放/倍速/进度）
│     ├─ linkdialog.h/.cpp            #   新建链路对话框
│     ├─ serialconsole.h/.cpp         #   载荷串口控制台（HDLC/Modbus/SLCAN 发送）
│     └─ messageinspector.h/.cpp      #   消息检查器
├─ tools/gen_mavlink.py               # 消息表生成器（事实源：官方 c_library_v2 头文件）
├─ third_party/                       # 官方参考材料（mavlink_ref/*.h 等）
├─ docs/                              # mavlink_msg_table.md / screenshot_*.png
└─ tests/                             # test_codec / test_integration / test_flightlog / shot_main
```

**分层职责**：UI ⇄ MavlinkEndpoint（协议引擎）⇄ LinkInterface（UDP/串口）⇄ 对端（真机 / 内置仿真器 / 载荷设备）。

---

## 四、协议正确性验证（自动化）

| 测试 | 内容 | 结果 |
|---|---|---|
| `tools/gen_mavlink.py` | 从官方 `c_library_v2` 提取字段序/CRC，交叉计算 crc_extra | **32/32 与官方一致**（含 MISSION_COUNT=221、MISSION_ITEM_INT=38、MISSION_ACK=153、PARAM_SET=168、PARAM_REQUEST_READ=214…） |
| `tests/test_codec.cpp` | ① 与 **pymavlink v20** 逐字节比对 ATTITUDE/HEARTBEAT/GLOBAL_POSITION_INT 帧 ② 解码 pymavlink 帧 ③ 往返 ④ 粘连/噪声流式解析 ⑤ 官方 CRC 抽样 | **13/13 通过** |
| `tests/test_integration.cpp` | 内置仿真器(UDP) → UdpLink → 协议引擎 → 状态聚合：心跳上线、遥测刷新、ARM ACK、STATUSTEXT、**起飞爬升**、**航点任务**（上传确认→3 航点依次到达→自动返航）、**参数管理**（读取 8 参数→PARAM_SET 修改→飞控确认回传） | **18/18 通过** |
| `tests/test_flightlog.cpp` | 飞行日志往返：遥测采样落盘（表头/行数/采样点）→ 回放恢复 VehicleState（位置/高度/模式/任务/电量） | **14/14 通过** |
| `tests/test_payload.cpp` | 载荷协议字节级比对：HDLC（CRC16-CCITT 已知向量/帧封装转义/回环解析/坏帧丢弃）、Modbus（CRC16-Modbus 已知向量/标准帧/发送自校验/30ms 静默粘包切帧）、SLCAN（标准/扩展帧收发往返） | **22/22 通过** |
| `tests/test_param_physics.cpp` | **参数驱动仿真物理**：MPC_Z_VEL_MAX=5 → 起飞爬升峰值 4.98m/s（突破默认 3.0）；MPC_XY_CRUISE=1 → 航点巡航峰值 1.00m/s；RTL_RETURN_ALT=35 → 返航爬升至 30.6m+ | **7/7 通过** |
| `tools/verify_pymavlink_gcs.py` | **官方协议栈互通**（pymavlink 模拟第三方地面站，连接 `sim_standalone`）：心跳识别 QUADROTOR/PX4、遥测流、ARM→ACK、8 参数读取、PARAM_SET 确认 5.5、2 航点任务上传 ACK | **15/15 通过** |

> 途中发现并修复的协议级坑：① 部署生态字段按类型长度降序（XML 声明序会全 MISMATCH）；② `GLOBAL_POSITION_INT` 已从 c_library_v2 master 移除、需按 PX4 锁定 XML 补充；③ 标量字段 count=0 导致的打包字节数为 0；④ pymavlink 按 MAVLink2 规则截断尾部零字节（COMMAND_LONG 的 confirmation=0），属合法行为。

---

## 五、构建与运行

### 环境
- Qt 6.x（5.15 亦可），编译器：MinGW 或 MSVC，CMake ≥ 3.16

### 构建
```bash
cmake -S SkyGCS -B SkyGCS/build -G Ninja -DCMAKE_PREFIX_PATH=<Qt安装目录>/<套件> -DCMAKE_BUILD_TYPE=Debug
cmake --build SkyGCS/build
```
（本机已验证构建矩阵：**MinGW × Qt 6.11.0**（主环境）、**MSVC 2026 × Qt 6.8.3**（`build-msvc-qt68`）、**MSVC 2026 × Qt 6.11.0**（`build-msvc-qt515`）——三种组合 8 个目标（skygcs + 6 测试 + sim_standalone）全部构建成功且测试通过。MSVC 下源码以 UTF-8 解析（CMake 已加 `/utf-8`，否则中文注释被按 GBK 误读导致语法错乱）。Qt 5.15 分支：CMake 含兼容 fallback、代码未用 Qt6-only API，但本机 kit 缺 Qt5Charts 组件未完整验证；Linux 需 CI 环境。）

### 参数驱动仿真物理

内置仿真器的控制律由 PX4 风格参数接管（非硬编码）：
- **MPC_Z_VEL_MAX** — 垂直速度上限（默认 3.0 m/s）
- **MPC_XY_CRUISE** — 航点巡航速度（默认 10.0，上限 4.0 m/s 级）
- **NAV_ACC_RAD** — 航点到达判定半径（默认 2.0 m）
- **RTL_RETURN_ALT** — 返航目标高度（默认 30.0 m）

在「参数管理」页修改任一参数即实时改变飞行行为（如把 MPC_XY_CRUISE 改为 1，航点巡航立即降至 1 m/s），由 `test_param_physics` 7 项断言闭环验证。

### 运行与体验（零硬件）
1. 启动 `build/skygcs.exe`。
2. 点击工具栏 **「▶ 启动内置仿真 (UDP)」** —— 地面站自动监听 14550，内置仿真器向该端口注入遥测。
3. 遥测监控页：姿态地平仪、高度/速度/电量实时变化 + 趋势曲线；**飞行指令**页点「解锁」→「起飞」（可设高度）→「返航」→「降落」，全程可看 ACK 与 STATUSTEXT；**任务规划**页预置 3 个示例航点，「上传任务」→「开始任务」即可看到飞行器依次飞越航点、任务完成自动返航；**消息检查器**页查看逐帧解码。
4. **参数管理**页点「读取参数列表」，仿真飞控回传 8 个 PX4 风格参数；双击「值」列修改（如 MPC_XY_CRUISE），回车后发送 PARAM_SET，飞控确认回传后数值变绿。
5. **飞行日志**页「开始记录」将遥测以 CSV 落盘（500ms 采样，默认 `logs/flight_*.csv`）；「打开…」选择任意日志即可回放（倍速 x0.5~x10、进度条），回放驱动全 UI 联动。
6. 串口体验：接 USB-转串口设备后，「新建串口链路」打开，在**载荷串口控制台**页可用 HDLC 帧 / Modbus RTU / SLCAN(CAN) 三种协议收发载荷帧。
7. 对接真实 PX4 SITL：`make px4_sitl gazebo` 启动后，新建 UDP 链路（本地 14550 / 目标 127.0.0.1:14550）即可。

### 测试
```bash
build/test_codec.exe          # MAVLink 协议交叉验证 (13)
build/test_integration.exe    # 端到端闭环（仿真↔地面站, 18）
build/test_flightlog.exe      # 飞行日志往返 (14)
build/test_payload.exe        # 载荷协议字节级 (22)
build/test_param_physics.exe  # 参数驱动仿真物理 (7)
# 第三方地面站互通（需 Python + pymavlink）:
build/sim_standalone.exe 45 &  python tools/verify_pymavlink_gcs.py   # (15)
```
> ⚠️ `test_integration` 与 `shot_main`/`sim_standalone` 都绑定 UDP 14550，**严禁并行运行**。

---

## 六、关键设计说明

1. **为什么自研 MAVLink 而非引第三方库**：面试/简历上"协议功底"是硬通货。自研编解码 + 官方头文件生成消息表 + 与 pymavlink 交叉验证，能完整讲清 CRC-extra、字段排序、v1/v2 差异等细节；生产环境仍建议用官方 c_library 或 MAVSDK。
2. **RS485/RS422 与 RS232 的软件差异**：电气层标准（差分信号/方向控制），软件层帧格式完全一致；半双工方向由硬件收发器依据 RTS/DE 自动控制，软件只需按全双工收发。RS485 典型载荷协议即本项目实现的 Modbus RTU；CAN 载荷通过串口 SLCAN 桥接（CANable 等适配器通用）。
3. **ACK 跟踪重发**：COMMAND_LONG 下发后 800ms 未确认重发（保留原始参数），最多 3 次，4s 超时放弃 —— 与真实地面站行为一致。
4. **UDP 链路策略**：绑定本地端口接收遥测、回复发往最近对端 —— 与 QGC 完全一致，可同时对接本仿真器与 PX4 SITL。

---

*SkyGCS — 调研、协议、架构、编码、验证、交付一体化实战项目。*
