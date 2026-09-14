# -*- coding: utf-8 -*-
"""检查 pymavlink 自带 common.xml 的消息定义与 crc_extra 计算差异"""
import os, re, sys
import pymavlink
from pymavlink.generator import mavparse

xml = os.path.join(os.path.dirname(pymavlink.__file__), 'dialects', 'v20', 'common.xml')
txt = open(xml, encoding='utf-8').read()

# 1) 列出消息
msgs = re.findall(r'<message id="(\d+)" name="([^"]+)"[^>]*>', txt)
print('消息总数:', len(msgs))
by_id = {}
for mid, name in msgs:
    by_id[int(mid)] = name
for mid in [0, 1, 2, 4, 11, 21, 22, 24, 30, 32, 33, 35, 74, 75, 76, 77, 93, 107, 109, 113, 115, 147, 242, 253]:
    print('id=%-4d %s' % (mid, by_id.get(mid, '--未定义--')))

# 2) 提取 id=1 的完整 XML 定义
m = re.search(r'<message id="1"[^>]*name="([^"]+)"[^>]*>.*?</message>', txt, re.S)
if m:
    print('\n=== id=1 完整定义 ===')
    print(m.group(0)[:1800])
