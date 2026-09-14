# -*- coding: utf-8 -*-
"""用 mavlink master 的官方 common.xml + pymavlink 生成器计算权威 crc_extra"""
import os
from pymavlink.generator import mavparse

xml = r'D:\Work-Qt\SkyGCS\third_party\common_master.xml'
parser = mavparse.MAVXML(xml)
by_name = {m.name: m for m in parser.message}

WANT = ['HEARTBEAT', 'SYS_STATUS', 'SYSTEM_TIME', 'PING', 'SET_MODE',
        'PARAM_REQUEST_LIST', 'PARAM_VALUE', 'GPS_RAW_INT', 'ATTITUDE',
        'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'RC_CHANNELS_RAW', 'VFR_HUD',
        'COMMAND_INT', 'COMMAND_LONG', 'COMMAND_ACK', 'HIL_SENSOR', 'HIL_GPS',
        'HIL_STATE_QUATERNION', 'HIL_ACTUATOR_CONTROLS', 'RADIO_STATUS',
        'BATTERY_STATUS', 'STATUSTEXT', 'HOME_POSITION']

print('%-22s %-6s %-8s %s' % ('消息', 'id', 'crc_extra', 'payload_len'))
for name in WANT:
    m = by_name.get(name)
    if m is None:
        print('%-22s 未找到' % name)
        continue
    # 计算 base payload 长度
    base_len = 0
    for f in m.ordered_fields[:m.base_fields()]:
        base_len += f.type_length * (f.array_length if f.array_length else 1)
    print('%-22s %-6d %-8d %d' % (name, m.id, m.crc_extra, base_len))
