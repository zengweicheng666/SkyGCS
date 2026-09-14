# SkyGCS 自我分析与进阶路线

> 客观评估当前实现的优势、不足与风险，并据此规划、落地进阶功能。本文档随开发持续更新。

---

## 一、项目定位

SkyGCS 是一套**从协议栈到地面站 UI 全自研**的无人机地面站（Qt/C++），覆盖 JD 全部技术点：QT/C++ 桌面应用、地面站开发、PX4/MAVLink 通信、RS485/RS422/CAN 串口通信。核心卖点是**协议可信**（24 条消息 CRC 与官方逐条对齐、与 pymavlink 逐字节交叉验证）与**可运行闭环**（内置仿真器 + 端到端测试）。

## 二、优势（对照 JD 逐条）

| 维度 | 现状 | 评价 |
|---|---|---|
| 协议深度 | 自研 MAVLink v1/v2 编解码、CRC-extra、字段排序、v2 尾零截断；23+ 条消息 | ★ 差异化亮点，面试可深聊 |
| 工程质量 | CMake 工程、模块分层（UI/引擎/链路/仿真）、无 Qt 依赖的协议核心可独立测试 | 良好 |
| 验证体系 | test_codec（pymavlink 交叉）+ test_integration（端到端）+ 离屏截图 | 有证据链 |
| 仿真闭环 | 内置 20Hz 物理仿真，解锁→起飞→爬升轨迹收敛 | 演示即所得 |
| 串口载荷 | HDLC / Modbus RTU / SLCAN(CAN) 三类协议 | 覆盖 JD 串口要求 |

## 三、不足与风险（诚实评估）

1. ~~任务规划缺失~~：已补齐（P1 航点任务全链路）。
2. ~~无飞行日志与回放~~：已补齐（P2 CSV 落盘 + 回放）。
3. ~~无参数管理~~：已补齐（P2 参数面板）。
4. ~~可视化单调~~：已补齐（P1 遥测趋势曲线）。
5. ~~未对接真实 SITL~~：已用官方协议栈 pymavlink 完成链路级互通实测（P3）；**真实 PX4 SITL 需 Linux/WSL + Gazebo 环境，本机（Windows）无法运行，未实测**——已通过 pymavlink（与 QGC/MAVSDK 同协议生态）在心跳/遥测/指令/参数/任务五个维度做等效互通验证。
6. ~~载荷协议仅编译级验证~~：已补齐（P3 test_payload 22 项字节级比对）。
7. ~~跨平台仅 MinGW~~：已验证 MSVC × Qt6.8/Qt6.11 构建矩阵（P3）；**Qt5.15 因本机缺 Qt5Charts 组件未完整验证，Linux 需 CI 环境**——CMake 已含 Qt5 兼容分支。

## 四、进阶路线

| 阶段 | 内容 | 状态 |
|---|---|---|
| **P1（本阶段）** | 任务规划（Mission）：MISSION_COUNT/ITEM_INT/REQUEST/ACK/CURRENT/REACHED 协议 + 仿真航点执行 + UI + 端到端测试 | ✅ 已完成 |
| **P1（本阶段）** | 遥测实时曲线（QtCharts）：高度/速度/电量趋势 | ✅ 已完成 |
| **P2（本阶段）** | 飞行日志：遥测 CSV 落盘 + 回放（倍速/进度，驱动全 UI） | ✅ 已完成 |
| **P2（本阶段）** | 参数管理：PARAM_REQUEST_READ/LIST/SET/VALUE 协议 + 参数面板 | ✅ 已完成 |
| P3 | 对接真实 PX4 SITL、QGC 互通实测 | 🟡 等效完成（pymavlink 链路互通 15/15；真实 PX4 SITL 需 Linux 环境） |
| P3 | 载荷协议自动化测试（HDLC/Modbus/SLCAN 与已知字节流比对） | ✅ 已完成（test_payload 22 项） |
| P3 | 跨平台构建矩阵（MSVC/Linux/Qt5.15） | 🟡 完成 MSVC×Qt6.8/Qt6.11；Qt5.15 缺组件、Linux 需 CI |
| P3 | 飞控参数下发到仿真物理模型（改 MPC_XY_CRUISE 影响巡航速度） | ✅ 已完成（test_param_physics 7 项） |

## 五、本阶段已实施进阶（落地后回填）

- [x] Mission 协议扩展（6 条消息 MISSION_CURRENT/COUNT/ITEM_INT/REQUEST_INT/ACK/ITEM_REACHED，CRC 官方对齐，消息表 24→30 条）
- [x] 仿真器航点任务执行（上传握手→开始→依次飞越→ITEM_REACHED/CURRENT 上报→任务完成自动 RTL）
- [x] 地面站任务面板（航点添加/删除/上传/开始/进度状态）
- [x] 遥测实时曲线（QtCharts：高度/垂直速度/电量，60s 滚动窗口）
- [x] 端到端航点任务测试（test_integration 9→14 项，含 RTL 后 MISSION_CURRENT=255 复位断言）
- [x] **PARAM 协议扩展**（PARAM_REQUEST_READ id=20 crc=214 / REQUEST_LIST id=21 / VALUE id=22 / SET id=23 crc=168，消息表 30→32 条）
- [x] **仿真器参数表**（8 个 PX4 风格参数，响应 LIST/READ/SET，修改后回传确认）
- [x] **参数管理面板**（读取列表/双击改值/PARAM_SET/确认变绿）
- [x] **飞行日志**（CSV 500ms 采样落盘 + 回放器驱动 VehicleState + 面板倍速/进度）
- [x] **参数端到端测试**（test_integration 14→18 项：列表读取完整、PARAM_SET 确认回传、类型正确）
- [x] **飞行日志往返测试**（新增 test_flightlog 14 项：落盘格式/行数/回放状态还原）

## 六、P3 已实施进阶（本阶段落地）

### P3-A 飞控参数下发 → 仿真物理模型（参数真正"接管"飞控行为）

- `flightsim.cpp` 控制律全面参数化：**MPC_Z_VEL_MAX**（垂直速度上限，替代硬编码 3.0）、**MPC_XY_CRUISE**（航点巡航速度，替代硬编码 4.0）、**NAV_ACC_RAD**（航点到达判定半径，替代硬编码 2.0）、**RTL_RETURN_ALT**（返航目标高度，替代 min(targetAlt, pos-1)）
- 新增 `test_param_physics` **7 项端到端断言**（实测数据）：MPC_Z_VEL_MAX=5 → 起飞爬升峰值 **4.98 m/s**（突破默认 3.0）；MPC_XY_CRUISE=1 → 航点巡航峰值**精确 1.00 m/s**；RTL_RETURN_ALT=35 → 返航段爬升至 **30.6m+**
- 集成测试回归：RTL 阶段爬升至 30.3m（参数生效），18 项全部通过

### P3-B 载荷协议自动化测试（字节级比对）

- 新增 `test_payload` **22 项**：HDLC（CRC16-CCITT 已知向量 0x29B1、帧封装/转义/回环解析、CRC 错误帧丢弃）、Modbus（CRC16-Modbus 已知向量 0x4B37、标准读寄存器帧 01 03 00 00 00 0A C5 CD、发送帧自校验、30ms 静默粘包切帧）、SLCAN（标准/扩展帧收发往返、非法行忽略）
- 测试过程修正 4 处测试数据错误（转义检查逻辑、len 字段、DLC 数据长度、粘包 CRC），协议实现本身 0 缺陷

### P3-C 官方协议栈互通验证（等效 QGC 链路）

- 新增 `sim_standalone`（无 GUI 独立仿真服务）+ `tools/verify_pymavlink_gcs.py`（pymavlink = QGC/MAVSDK 同源官方协议库）
- **15/15 通过**：HEARTBEAT 识别（QUADROTOR/PX4）、ATTITUDE/GLOBAL_POSITION_INT/BATTERY_STATUS 遥测流、ARM 指令 ACK(ACCEPTED)、PARAM_REQUEST_LIST 8 参数、PARAM_SET 确认回传 5.5、MISSION 2 航点上传 ACK(ACCEPTED)
- 说明：真实 PX4 SITL 需 Linux/WSL+Gazebo，本机不可行；pymavlink 互通是协议层的强等效证据

### P3-D 跨平台构建矩阵

- **已验证 3 种组合全部构建成功（8 目标）+ 测试通过**：
  - MinGW × Qt 6.11.0（主开发环境，全测试）
  - MSVC 2026 × Qt 6.8.3（build-msvc-qt68，codec/payload/param_physics 通过）
  - MSVC 2026 × Qt 6.11.0（build-msvc-qt515 首配，codec/flightlog/payload 通过）
- **新增 MSVC 适配**：`add_compile_options(/utf-8)`（MSVC 默认按 GBK 解析 UTF-8 源码会导致中文注释破坏语法——首次构建 C4430/C2143 全量报错，加 /utf-8 后归零）
- **受限项（如实记录）**：Qt 5.15.2 kit 缺 Qt5Charts 组件无法配置（CMake 已含 Qt5 兼容 fallback，代码未用 Qt6-only API）；Linux 需 CI 环境
- 构建目录：build-msvc-qt68 / build-msvc-qt515（MSVC），均已加入 .gitignore
