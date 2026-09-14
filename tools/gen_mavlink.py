# -*- coding: utf-8 -*-
"""
SkyGCS MAVLink 消息表生成器 (v2)

事实源:
  1. github.com/mavlink/c_library_v2 官方生成头文件 (third_party/mavlink_ref/*.h)
     —— 部署生态 (QGC/MAVSDK/PX4 发布版) 实际使用的字段序(按类型长度降序排列) 与 CRC_EXTRA
  2. HEARTBEAT(在 minimal.xml)、GLOBAL_POSITION_INT(在 standard.xml) 按官方定义补充

流程:
  - 从官方头文件提取 结构体字段序 / CRC / LEN / MIN_LEN
  - 用 mavlink C 库同款 CRC-16/MCRF4XX 算法计算 crc_extra 并与官方 CRC 交叉验证
  - 生成 src/mavlink/mavlink_generated.h 与 docs/mavlink_msg_table.md

用法: python tools/gen_mavlink.py
"""
import argparse, glob, os, re, sys, datetime

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
REF_DIR = os.path.join(ROOT, 'third_party', 'mavlink_ref')

# ---------------------------------------------------------------------------
# CRC-16/MCRF4XX —— 与 mavlink C 库 crc_accumulate 完全一致
# ---------------------------------------------------------------------------
def crc_accumulate(data, crc):
    tmp = data ^ (crc & 0xff)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF

class X25CRC:
    def __init__(self):
        self.crc = 0xFFFF
    def accumulate(self, buf):
        if type(buf) is str:
            buf = buf.encode()
        for b in buf:
            self.crc = crc_accumulate(b, self.crc)
    def accumulate_str(self, s):
        self.accumulate(s)

def crc_extra(name, fields):
    """与 pymavlink message_checksum 一致: 字段类型用完整 C 类型名;
       数组长度按单字节累加; 返回 (crc&0xFF)^(crc>>8)
       fields: (ma_type, c_type, name, count)"""
    crc = X25CRC()
    crc.accumulate_str(name + ' ')
    for ma, ct, n, cnt in fields:
        crc.accumulate_str(ct + ' ')
        crc.accumulate_str(n + ' ')
        if cnt:
            crc.accumulate([cnt])
    return (crc.crc & 0xFF) ^ (crc.crc >> 8)

C_TYPE_MAP = {
    'uint8_t': 'u8', 'int8_t': 'i8', 'uint16_t': 'u16', 'int16_t': 'i16',
    'uint32_t': 'u32', 'int32_t': 'i32', 'uint64_t': 'u64', 'float': 'f32', 'char': 'char',
}
SZ = {'u8': 1, 'i8': 1, 'char': 1, 'u16': 2, 'i16': 2, 'u32': 4, 'i32': 4, 'u64': 8, 'f32': 4}

# ---------------------------------------------------------------------------
# 从官方头文件提取
# ---------------------------------------------------------------------------
def extract_from_header(path):
    txt = open(path, encoding='utf-8', errors='replace').read()
    mm = re.search(r'#define MAVLINK_MSG_ID_(\w+)\s+(\d+)', txt)
    if not mm:
        raise RuntimeError('无法识别消息宏: ' + path)
    name, mid = mm.group(1), int(mm.group(2))
    crc = int(re.search(r'#define MAVLINK_MSG_ID_%s_CRC\s+(\d+)' % re.escape(name), txt).group(1))
    wlen = int(re.search(r'#define MAVLINK_MSG_ID_%s_LEN\s+(\d+)' % re.escape(name), txt).group(1))
    ml = re.search(r'#define MAVLINK_MSG_ID_%s_MIN_LEN\s+(\d+)' % re.escape(name), txt)
    min_len = int(ml.group(1)) if ml else wlen

    sm = re.search(r'typedef struct __mavlink_%s_t\s*\{(.*?)\}\s*\)?\s*mavlink_%s_t;'
                   % (name.lower(), name.lower()), txt, re.S)
    if not sm:
        raise RuntimeError('无法解析结构体: ' + path)
    fields = []
    for m in re.finditer(r'(uint\w+_t|int\w+_t|float|char)\s+(\w+)(?:\s*\[(\d+)\])?', sm.group(1)):
        ct, n, cnt = m.group(1), m.group(2), m.group(3)
        if ct not in C_TYPE_MAP:
            raise RuntimeError('未知类型 %s in %s.%s' % (ct, name, n))
        fields.append((C_TYPE_MAP[ct], ct, n, int(cnt) if cnt else 0))
    return name, mid, crc, wlen, min_len, fields


def base_fields_of(fields, min_len):
    """截取覆盖 min_len 字节的基础字段 (扩展字段不参与 CRC, 也不进入消息表)"""
    base, acc = [], 0
    for ma, ct, n, cnt in fields:
        sz = SZ[ma] * (cnt or 1)
        if acc + sz <= min_len:
            base.append((ma, ct, n, cnt or 0))
            acc += sz
        else:
            break
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    msgs = []
    headers = sorted(glob.glob(os.path.join(REF_DIR, '*.h')))
    if not headers:
        print('[ERR] 未找到官方头文件目录: %s' % REF_DIR)
        return 1
    for h in headers:
        if 'global_position_int' in h or 'heartbeat' in h:   # 手动补充
            continue
        msgs.append(extract_from_header(h))

    # 手动补充: HEARTBEAT / GLOBAL_POSITION_INT (minimal.xml / standard.xml, 排序序)
    # 排序规则: 按类型长度降序 (稳定排序), 即部署生态的 wire 序
    def sorted_fields(fields):
        return sorted(fields, key=lambda f: SZ[f[0]], reverse=True)

    def f4(ma, name, cnt=0):
        return (ma, {'u8': 'uint8_t', 'i8': 'int8_t', 'u16': 'uint16_t', 'i16': 'int16_t',
                     'u32': 'uint32_t', 'i32': 'int32_t', 'u64': 'uint64_t', 'f32': 'float',
                     'char': 'char'}[ma], name, cnt)

    msgs.append(('HEARTBEAT', 0, 50, 9, 9, sorted_fields([
        f4('u8', 'type'), f4('u8', 'autopilot'), f4('u8', 'base_mode'),
        f4('u32', 'custom_mode'), f4('u8', 'system_status'), f4('u8', 'mavlink_version')])))
    msgs.append(('GLOBAL_POSITION_INT', 33, 104, 28, 28, sorted_fields([
        f4('u32', 'time_boot_ms'), f4('i32', 'lat'), f4('i32', 'lon'), f4('i32', 'alt'),
        f4('i32', 'relative_alt'), f4('i16', 'vx'), f4('i16', 'vy'), f4('i16', 'vz'),
        f4('u16', 'hdg')])))
    msgs.sort(key=lambda m: m[1])

    # 交叉验证 (仅基础字段参与 CRC)
    print('%-24s %-5s %-6s %-6s %-6s %s' % ('消息', 'id', 'CRC', 'LEN', 'MIN', '字段数'))
    ok = True
    for name, mid, crc, wlen, min_len, fields in msgs:
        base = base_fields_of(fields, min_len)
        calc = crc_extra(name, base)
        match = 'OK' if calc == crc else 'MISMATCH'
        if calc != crc:
            ok = False
        print('  [%s] %-24s %-5d %-6d %-6d %-6d %d' % (match, name, mid, crc, wlen, min_len, len(base)))
    if not ok:
        print('\n[FAIL] 存在不一致!')
        return 1
    print('\n[PASS] %d 条消息 crc_extra 全部与官方一致' % len(msgs))

    # 生成 C++ 头文件
    out_path = args.out or os.path.join(ROOT, 'src', 'mavlink', 'mavlink_generated.h')
    L = []
    L.append('// ============================================================================')
    L.append('// 本文件由 tools/gen_mavlink.py 自动生成 —— 请勿手工修改')
    L.append('// 字段序/CRC/LEN 提取自官方 c_library_v2 (github.com/mavlink/c_library_v2)')
    L.append('// 已按 mavlink C 库 crc_accumulate 算法交叉验证')
    L.append('// 生成时间: %s' % datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    L.append('// ============================================================================')
    L.append('#pragma once')
    L.append('#include <cstdint>')
    L.append('#include <cstddef>')
    L.append('#include "mavlink_types.h"')
    L.append('')
    L.append('namespace skygcs {')
    L.append('')
    L.append('// 消息 ID')
    L.append('enum MavMsgId : uint32_t {')
    for name, mid, *_ in msgs:
        L.append('    MAV_MSG_ID_%s = %d,' % (name, mid))
    L.append('};')
    L.append('')
    for name, mid, crc, wlen, min_len, fields in msgs:
        base = base_fields_of(fields, min_len)
        off = 0
        L.append('// %s: id=%d, crc_extra=%d, payload=%dB' % (name, mid, crc, wlen))
        L.append('static constexpr FieldDef kFields_%s[] = {' % name)
        for ma, ct, n, cnt in base:
            arr = ', %d' % cnt if cnt else ''
            L.append('    { FieldType::%s, %d, %d%s },' % (ma.upper(), off, SZ[ma], arr))
            off += SZ[ma] * (cnt or 1)
        L.append('};')
        L.append('')
    L.append('// 消息定义表')
    L.append('static constexpr MsgDef kMsgDefs[] = {')
    for name, mid, crc, wlen, min_len, fields in msgs:
        base = base_fields_of(fields, min_len)
        L.append('    { %d, "%s", %d, %d, %d, kFields_%s, %d },' % (
            mid, name, crc, min_len, wlen, name, len(base)))
    L.append('};')
    L.append('static constexpr size_t kMsgDefCount = %d;' % len(msgs))
    L.append('')
    L.append('} // namespace skygcs')
    L.append('')
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(L))
    print('\n[GEN] 已生成: %s' % out_path)

    md_path = os.path.join(ROOT, 'docs', 'mavlink_msg_table.md')
    os.makedirs(os.path.dirname(md_path), exist_ok=True)
    with open(md_path, 'w', encoding='utf-8') as f:
        f.write('| 消息 | ID | CRC_EXTRA | 基础载荷(B) | 完整载荷(B) |\n|---|---|---|---|---|\n')
        for name, mid, crc, wlen, min_len, _ in msgs:
            f.write('| %s | %d | %d | %d | %d |\n' % (name, mid, crc, min_len, wlen))
    print('[GEN] 已生成: %s' % md_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
