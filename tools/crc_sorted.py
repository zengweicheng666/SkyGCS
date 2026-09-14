# -*- coding: utf-8 -*-
"""用 sort_fields=True 重新计算权威 crc_extra"""
import os
from pymavlink.generator import mavparse

base = r'D:\Work-Qt\SkyGCS\third_party'
parser = mavparse.MAVXML(os.path.join(base, 'common.xml'), wire_protocol_version='2.0')
# sort_fields 是实例属性, 解析后需重建 ordered_fields
for m in parser.message:
    if m.base_fields() > 0 and m.fields:
        m.ordered_fields = sorted(m.fields[:m.base_fields()],
                                  key=lambda f: f.type_length, reverse=True)
        m.ordered_fields.extend(m.fields[m.base_fields():])

by_name = {m.name: m for m in parser.message}
WANT = ['HEARTBEAT', 'SYS_STATUS', 'SYSTEM_TIME', 'PING', 'SET_MODE',
        'PARAM_REQUEST_LIST', 'PARAM_VALUE', 'GPS_RAW_INT', 'ATTITUDE',
        'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'RC_CHANNELS_RAW', 'VFR_HUD',
        'COMMAND_INT', 'COMMAND_LONG', 'COMMAND_ACK', 'HIL_SENSOR', 'HIL_GPS',
        'HIL_STATE_QUATERNION', 'HIL_ACTUATOR_CONTROLS', 'RADIO_STATUS',
        'BATTERY_STATUS', 'STATUSTEXT', 'HOME_POSITION']

print('%-24s %-5s %-6s %-8s' % ('消息', 'id', 'crc', 'base_len'))
for name in WANT:
    m = by_name.get(name)
    if m is None:
        print('%-24s 未找到' % name)
        continue
    # 重新计算 crc_extra (message_checksum 使用 ordered_fields)
    from pymavlink.generator.mavcrc import x25crc
    crc = x25crc()
    crc.accumulate_str(m.name + ' ')
    for i in range(m.base_fields()):
        f = m.ordered_fields[i]
        crc.accumulate_str(f.type + ' ')
        crc.accumulate_str(f.name + ' ')
        if f.array_length:
            crc.accumulate([f.array_length])
    ce = (crc.crc & 0xFF) ^ (crc.crc >> 8)
    base_f = m.ordered_fields[:m.base_fields()]
    base_len = sum(f.type_length * (f.array_length if f.array_length else 1) for f in base_f)
    print('%-24s %-5d %-6d %-8d' % (name, m.id, ce, base_len))
