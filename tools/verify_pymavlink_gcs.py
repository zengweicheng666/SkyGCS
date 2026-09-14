# -*- coding: utf-8 -*-
"""
SkyGCS ↔ 官方协议栈 (pymavlink) 实时互通验证
=================================================
用 pymavlink (QGC/MAVSDK 同源的官方 MAVLink 协议库) 作为"第三方地面站"连接
SkyGCS 内置仿真器 (sim_standalone.exe, UDP 14550), 逐项验证:

  1. HEARTBEAT 识别: PX4 / QUADROTOR / 心跳稳定
  2. 遥测流:        ATTITUDE / GLOBAL_POSITION_INT / BATTERY_STATUS 持续到达
  3. 指令闭环:      COMMAND_LONG(ARM) → 收到 COMMAND_ACK(ACCEPTED)
  4. 参数读写:      PARAM_REQUEST_LIST → 8 个 PARAM_VALUE; PARAM_SET 后回传确认
  5. 任务协议:      MISSION_COUNT/ITEM_INT 上传 → MISSION_ACK(ACCEPTED)

等价于真实 QGroundControl 链路验证的协议层证据 (pymavlink 与 QGC 同一协议生态)。

用法: python tools/verify_pymavlink_gcs.py
依赖: pymavlink (pip install pymavlink)
"""
import subprocess
import sys
import time

from pymavlink import mavutil

BUILD = r"D:\Work-Qt\SkyGCS\build\sim_standalone.exe"
PORT = 14550

PASS = 0
FAIL = 0


def check(ok, what, detail=""):
    global PASS, FAIL
    tag = "PASS" if ok else "FAIL"
    if ok:
        PASS += 1
    else:
        FAIL += 1
    print("[%s] %s %s" % (tag, what, detail))
    sys.stdout.flush()


def main():
    print("== 启动 SkyGCS 内置仿真 (sim_standalone) ==")
    proc = subprocess.Popen([BUILD, "45"], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)

    time.sleep(1.0)
    print("== pymavlink 连接 udpin:0.0.0.0:%d ==" % PORT)
    conn = mavutil.mavlink_connection("udpin:0.0.0.0:%d" % PORT)

    # ---- 1. 心跳识别 ----
    hb = None
    for _ in range(30):
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg is not None:
            hb = msg
            break
    check(hb is not None, "收到 HEARTBEAT")
    if hb is None:
        proc.terminate()
        return 1
    check(hb.type == mavutil.mavlink.MAV_TYPE_QUADROTOR, "飞行器类型=QUADROTOR",
          "(type=%d)" % hb.type)
    check(hb.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4, "飞控=AutopilotPX4",
          "(autopilot=%d)" % hb.autopilot)

    # ---- 2. 遥测流 ----
    got = {"ATTITUDE": 0, "GLOBAL_POSITION_INT": 0, "BATTERY_STATUS": 0}
    end = time.time() + 4.0
    while time.time() < end:
        msg = conn.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() in got:
            got[msg.get_type()] += 1
    check(got["ATTITUDE"] >= 10, "ATTITUDE 持续到达", "(n=%d)" % got["ATTITUDE"])
    check(got["GLOBAL_POSITION_INT"] >= 3, "GLOBAL_POSITION_INT 持续到达",
          "(n=%d)" % got["GLOBAL_POSITION_INT"])
    check(got["BATTERY_STATUS"] >= 1, "BATTERY_STATUS 到达", "(n=%d)" % got["BATTERY_STATUS"])

    # ---- 3. 指令闭环: ARM ----
    conn.mav.command_long_send(
        1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
    check(ack is not None, "ARM 收到 COMMAND_ACK")
    if ack is not None:
        check(ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
              and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED,
              "ARM ACK = ACCEPTED", "(cmd=%d result=%d)" % (ack.command, ack.result))

    # ---- 4. 参数读写 ----
    conn.mav.param_request_list_send(1, 1)
    params = {}
    end = time.time() + 3.0
    while time.time() < end and len(params) < 8:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None:
            pid = msg.param_id.rstrip("\x00")
            params[pid] = msg.param_value
    check(len(params) == 8, "PARAM_REQUEST_LIST 收到 8 个参数", "(n=%d)" % len(params))
    check("MPC_XY_CRUISE" in params, "参数含 MPC_XY_CRUISE")
    if "MPC_XY_CRUISE" in params:
        check(abs(params["MPC_XY_CRUISE"] - 10.0) < 1e-3, "MPC_XY_CRUISE 初始值=10.0")

    # PARAM_SET: 修改 MPC_XY_CRUISE → 飞控回传确认
    conn.mav.param_set_send(1, 1, b"MPC_XY_CRUISE", 5.5, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    confirmed = None
    end = time.time() + 2.0
    while time.time() < end:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None:
            pid = msg.param_id.rstrip("\x00")
            if pid == "MPC_XY_CRUISE":
                confirmed = msg.param_value
                break
    check(confirmed is not None, "PARAM_SET 收到确认回传")
    if confirmed is not None:
        check(abs(confirmed - 5.5) < 1e-3, "PARAM_SET 值确认=5.5", "(got=%.3f)" % confirmed)

    # ---- 5. 任务上传 ----
    conn.mav.mission_count_send(1, 1, 2, 0)
    for i, (x, y, z) in enumerate([(24.510539, 117.650000, 8.0),
                                   (24.511078, 117.650592, 8.0)]):
        req = conn.recv_match(type="MISSION_REQUEST_INT", blocking=True, timeout=3)
        if req is None:
            check(False, "收到 MISSION_REQUEST_INT(seq=%d)" % i)
            break
        conn.mav.mission_item_int_send(
            1, 1, i, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
            0, 0, 0, 0, int(x * 1e7), int(y * 1e7), z, 0)
    ack = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=3)
    check(ack is not None, "任务上传收到 MISSION_ACK")
    if ack is not None:
        check(ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED,
              "MISSION_ACK = ACCEPTED", "(type=%d)" % ack.type)

    # ---- 汇总 ----
    proc.terminate()
    print()
    print("== 结果: %s (%d PASS, %d FAIL) =="
          % ("全部通过" if FAIL == 0 else "存在失败", PASS, FAIL))
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
