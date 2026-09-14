# -*- coding: utf-8 -*-
import re

txt = open(r'D:\Work-Qt\SkyGCS\third_party\common_1.0.12.xml', encoding='utf-8').read()
m = re.search(r'<message id="76"[^>]*>(.*?)</message>', txt, re.S)
if m:
    print('=== 1.0.12 COMMAND_LONG ===')
    print(m.group(1)[:1400])

# 也看看 1.0.12 发布后 common.xml 是否还有 "sorted" 等属性
print('\n=== message 标签属性示例 ===')
for mm in re.finditer(r'<message id="\d+"[^>]*>', txt):
    line = mm.group(0)
    if any(k in line for k in ('sorted', 'order', 'reorder')):
        print(line[:200])
print('(无 sorted/reorder 属性)')

# 检查版本号
mv = re.search(r'<version>(\d+)</version>', txt)
print('\nversion tag:', mv.group(1) if mv else 'none')
