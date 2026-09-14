# -*- coding: utf-8 -*-
"""多角度验证 SYS_STATUS/COMMAND_LONG 等消息的 CRC_EXTRA"""
import os, sys, glob

def crc_accumulate(data, crc):
    """mavlink C 库 crc_accumulate (逐字节)"""
    tmp = data ^ (crc & 0xff)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

def checksum(name, fields, array_style='byte', return_style='low'):
    crc = 0xFFFF
    def acc_bytes(bs):
        nonlocal crc
        for b in bs:
            crc = crc_accumulate(b, crc)
    def acc_str(s):
        acc_bytes(s.encode())
    acc_str(name + ' ')
    for f in fields:
        t, n = f[0], f[1]
        alen = f[2] if len(f) > 2 else 0
        acc_str(t + ' ')
        acc_str(n + ' ')
        if alen:
            if array_style == 'byte':
                acc_bytes([alen])
            else:
                acc_str(str(alen))
    if return_style == 'low':
        return crc & 0xFF
    else:
        return (crc & 0xFF) ^ (crc >> 8)

SYS_STATUS_FIELDS = [
    ('uint32_t', 'onboard_control_sensors_present'), ('uint32_t', 'onboard_control_sensors_enabled'),
    ('uint32_t', 'onboard_control_sensors_health'), ('uint16_t', 'load'),
    ('uint16_t', 'voltage_battery'), ('int16_t', 'current_battery'), ('int8_t', 'battery_remaining'),
    ('uint16_t', 'drop_rate_comm'), ('uint16_t', 'errors_comm'),
    ('uint16_t', 'errors_count1'), ('uint16_t', 'errors_count2'),
    ('uint16_t', 'errors_count3'), ('uint16_t', 'errors_count4'),
]
COMMAND_LONG_FIELDS = [
    ('float','param1'),('float','param2'),('float','param3'),('float','param4'),
    ('float','param5'),('float','param6'),('float','param7'),
    ('uint16_t','command'),('uint8_t','target_system'),('uint8_t','target_component'),('uint8_t','confirmation'),
]
PARAM_VALUE_FIELDS = [('char','param_id',16),('float','param_value'),('uint8_t','param_type'),('uint16_t','param_count'),('uint16_t','param_index')]
GPS_RAW_INT_FIELDS = [
    ('uint32_t','time_boot_ms'),('int32_t','lat'),('int32_t','lon'),('int32_t','alt'),
    ('uint16_t','eph'),('uint16_t','epv'),('uint16_t','vel'),('uint16_t','cog'),
    ('uint8_t','fix_type'),('uint8_t','satellites_visible'),
]

cases = [
    ('SYS_STATUS', SYS_STATUS_FIELDS, 124),
    ('COMMAND_LONG', COMMAND_LONG_FIELDS, 152),
    ('PARAM_VALUE', PARAM_VALUE_FIELDS, 220),
    ('GPS_RAW_INT', GPS_RAW_INT_FIELDS, 24),
]
print('%-14s %-18s %-18s %s' % ('消息', 'byte+low', 'byte+low^high', '预期'))
for name, fields, expect in cases:
    v1 = checksum(name, fields, 'byte', 'low')
    v2 = checksum(name, fields, 'byte', 'x')
    print('%-14s %-18d %-18d %s' % (name, v1, v2, expect))

# 搜索磁盘上的 MAVLink 生成头文件作为权威参照
print('\n磁盘上已存在的 mavlink 头文件:')
hits = []
for root in ['C:\\', 'D:\\']:
    for pat in ['mavlink_msg_sys_status.h', 'mavlink_msg_heartbeat.h']:
        try:
            hits += glob.glob(os.path.join(root, '**', pat), recursive=True)[:5]
        except Exception:
            pass
for h in hits[:10]:
    print(' ', h)
