# -*- coding: utf-8 -*-
"""用 PX4 锁定的 mavlink 提交(1b1e896)计算所有消息的权威 crc_extra 与字段"""
import os, re
from pymavlink.generator import mavparse

base = r'D:\Work-Qt\SkyGCS\third_party'
# 需要同目录 includes
os.replace(os.path.join(base, 'common_px4.xml'), os.path.join(base, 'common.xml'))
os.replace(os.path.join(base, 'minimal_px4.xml'), os.path.join(base, 'minimal.xml'))
os.replace(os.path.join(base, 'standard_px4.xml'), os.path.join(base, 'standard.xml'))

parser = mavparse.MAVXML(os.path.join(base, 'common.xml'))
by_name = {m.name: m for m in parser.message}

WANT = ['HEARTBEAT', 'SYS_STATUS', 'SYSTEM_TIME', 'PING', 'SET_MODE',
        'PARAM_REQUEST_LIST', 'PARAM_VALUE', 'GPS_RAW_INT', 'ATTITUDE',
        'LOCAL_POSITION_NED', 'GLOBAL_POSITION_INT', 'RC_CHANNELS_RAW', 'VFR_HUD',
        'COMMAND_INT', 'COMMAND_LONG', 'COMMAND_ACK', 'HIL_SENSOR', 'HIL_GPS',
        'HIL_STATE_QUATERNION', 'HIL_ACTUATOR_CONTROLS', 'RADIO_STATUS',
        'BATTERY_STATUS', 'STATUSTEXT', 'HOME_POSITION']

out = []
print('%-24s %-5s %-6s %-8s' % ('消息', 'id', 'crc', 'base_len'))
for name in WANT:
    m = by_name.get(name)
    if m is None:
        print('%-24s 未找到' % name)
        continue
    base = m.ordered_fields[:m.base_fields()]
    base_len = sum(f.type_length * (f.array_length if f.array_length else 1) for f in base)
    print('%-24s %-5d %-6d %-8d' % (name, m.id, m.crc_extra, base_len))
    out.append((name, m.id, m.crc_extra, base_len,
                [(f.type, f.name, f.array_length) for f in base]))

with open(os.path.join(base, 'px4_defs.txt'), 'w', encoding='utf-8') as f:
    for name, mid, crc, blen, fields in out:
        f.write('%s|%d|%d|%d|%s\n' % (name, mid, crc, blen,
                ';'.join('%s:%s:%d' % (t, n, a) for t, n, a in fields)))
print('\n已写入 px4_defs.txt')
