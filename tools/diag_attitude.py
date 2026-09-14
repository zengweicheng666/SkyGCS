# -*- coding: utf-8 -*-
"""临时诊断: 检查仿真器 ATTITUDE 的 roll/pitch 在飞行中是否变化"""
import subprocess, sys, time
from pymavlink import mavutil

BUILD = r"D:\Work-Qt\SkyGCS\build\sim_standalone.exe"
proc = subprocess.Popen([BUILD, "40"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
time.sleep(1.0)
conn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
conn.wait_heartbeat(timeout=5)

max_roll = max_pitch = 0.0
samples = 0

def collect(seconds, tag):
    global max_roll, max_pitch, samples
    end = time.time() + seconds
    while time.time() < end:
        msg = conn.recv_match(type="ATTITUDE", blocking=True, timeout=1)
        if msg:
            max_roll = max(max_roll, abs(msg.roll) * 57.2958)
            max_pitch = max(max_pitch, abs(msg.pitch) * 57.2958)
            samples += 1
    print("[%s] roll峰值=%.2f° pitch峰值=%.2f° (n=%d)" % (tag, max_roll, max_pitch, samples))
    sys.stdout.flush()

# 阶段1: 悬停(未起飞)
collect(3, "悬停")

# ARM + 起飞
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(0.5)
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 8)
collect(4, "起飞爬升")

# 1 航点(北 25m) + 开始任务
conn.mav.mission_count_send(1, 1, 1, 0)
req = conn.recv_match(type="MISSION_REQUEST_INT", blocking=True, timeout=3)
conn.mav.mission_item_int_send(1, 1, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                               mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1,
                               0, 0, 0, 0, int(24.510539 * 1e7), int(117.650000 * 1e7), 8, 0)
conn.recv_match(type="MISSION_ACK", blocking=True, timeout=3)
conn.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
collect(6, "水平飞行")

proc.terminate()
print("结论:", "姿态在变化" if max_roll > 0.5 or max_pitch > 0.5 else "姿态始终为0!")
