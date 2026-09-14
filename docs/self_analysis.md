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

1. **任务规划缺失**：地面站标配能力（航点任务上传/执行/进度）未实现 —— 本阶段优先补齐。
2. **无飞行日志与回放**：遥测落盘（tlog/CSV）与回放是工程型地面站的标配，缺失。
3. **无参数管理**：PARAM_REQUEST_LIST/PARAM_SET 未实现（JD 常见要求）。
4. **可视化单调**：遥测以数值为主，缺实时趋势曲线。
5. **未对接真实 SITL**：与 PX4 SITL / QGC 的互通性未实测（协议级已验证，链路级待验）。
6. **测试覆盖**：串口链路（SerialLink）、载荷协议（HDLC/Modbus/SLCAN）仅编译级验证，无自动化协议测试。
7. **跨平台**：仅在 Windows/Qt6 MinGW 验证，未验证 MSVC / Linux / Qt5.15。

## 四、进阶路线

| 阶段 | 内容 | 状态 |
|---|---|---|
| **P1（本阶段）** | 任务规划（Mission）：MISSION_COUNT/ITEM_INT/REQUEST/ACK/CURRENT/REACHED 协议 + 仿真航点执行 + UI + 端到端测试 | ✅ 已完成 |
| **P1（本阶段）** | 遥测实时曲线（QtCharts）：高度/速度/电量趋势 | ✅ 已完成 |
| **P2（本阶段）** | 飞行日志：遥测 CSV 落盘 + 回放（倍速/进度，驱动全 UI） | ✅ 已完成 |
| **P2（本阶段）** | 参数管理：PARAM_REQUEST_READ/LIST/SET/VALUE 协议 + 参数面板 | ✅ 已完成 |
| P3 | 对接真实 PX4 SITL、QGC 互通实测 | 待办 |
| P3 | 载荷协议自动化测试（HDLC/Modbus/SLCAN 与已知字节流比对） | 待办 |
| P3 | 跨平台构建矩阵（MSVC/Linux/Qt5.15） | 待办 |
| P3 | 飞控参数下发到仿真物理模型（改 MPC_XY_CRUISE 影响巡航速度） | 待办 |

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
