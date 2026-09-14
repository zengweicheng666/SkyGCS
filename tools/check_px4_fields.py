# -*- coding: utf-8 -*-
import re

txt = open(r'D:\Work-Qt\SkyGCS\third_party\common.xml', encoding='utf-8').read()
for mid, name in [('1', 'SYS_STATUS'), ('24', 'GPS_RAW_INT'), ('76', 'COMMAND_LONG'), ('74', 'VFR_HUD'), ('22', 'PARAM_VALUE')]:
    m = re.search(r'<message id="%s" name="%s"[^>]*>(.*?)</message>' % (mid, name), txt, re.S)
    if not m:
        print('== %s 未找到 ==' % name)
        continue
    body = m.group(1)
    fields = re.findall(r'<field type="([^"]+)" name="([^"]+)"', body)
    print('== %s (%d fields) ==' % (name, len(fields)))
    for t, n in fields:
        print('   %-12s %s' % (t, n))

# 检查 include
print('\nincludes:', re.findall(r'<include>([^<]+)</include>', txt))
# 检查 minimal.xml 中的 HEARTBEAT
mtxt = open(r'D:\Work-Qt\SkyGCS\third_party\minimal.xml', encoding='utf-8').read()
m = re.search(r'<message id="0" name="HEARTBEAT"[^>]*>(.*?)</message>', mtxt, re.S)
if m:
    print('\nminimal.xml HEARTBEAT fields:')
    for t, n in re.findall(r'<field type="([^"]+)" name="([^"]+)"', m.group(1)):
        print('   %-12s %s' % (t, n))
# standard.xml 中的 GLOBAL_POSITION_INT
stxt = open(r'D:\Work-Qt\SkyGCS\third_party\standard.xml', encoding='utf-8').read()
m = re.search(r'<message id="33" name="GLOBAL_POSITION_INT"[^>]*>(.*?)</message>', stxt, re.S)
if m:
    print('\nstandard.xml GLOBAL_POSITION_INT fields:')
    for t, n in re.findall(r'<field type="([^"]+)" name="([^"]+)"', m.group(1)):
        print('   %-12s %s' % (t, n))
else:
    print('\nstandard.xml 中无 GLOBAL_POSITION_INT')
