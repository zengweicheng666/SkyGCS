# -*- coding: utf-8 -*-
"""从 mavlink 1.0.12 官方 XML 计算 GLOBAL_POSITION_INT 等缺失消息的权威定义"""
import re, os
from pymavlink.generator import mavparse

xml = r'D:\Work-Qt\SkyGCS\third_party\common_1.0.12.xml'
# 1.0.12 的 common.xml 可能 include minimal.xml, 但该版本 XML 内嵌 minimal 消息?
parser = mavparse.MAVXML(xml)
by_name = {m.name: m for m in parser.message}

for name in ['GLOBAL_POSITION_INT', 'HEARTBEAT', 'SYS_STATUS', 'GPS_RAW_INT', 'PARAM_VALUE',
             'SET_MODE', 'VFR_HUD', 'RADIO_STATUS', 'HIL_GPS', 'HIL_ACTUATOR_CONTROLS',
             'RC_CHANNELS_RAW']:
    m = by_name.get(name)
    if m is None:
        print('%-22s 未找到' % name)
        continue
    fields = [(f.type, f.name, f.array_length) for f in m.ordered_fields[:m.base_fields()]]
    base_len = sum(f.type_length * (f.array_length if f.array_length else 1) for f in m.ordered_fields[:m.base_fields()])
    print('%-22s id=%-4d crc=%-4d base_len=%d' % (name, m.id, m.crc_extra, base_len))
    print('    ', fields)
