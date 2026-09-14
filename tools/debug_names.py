# -*- coding: utf-8 -*-
import glob, os, re
for h in sorted(glob.glob(r'third_party\mavlink_ref\*.h')):
    txt = open(h, encoding='utf-8', errors='replace').read()
    mn = re.search(r'<message name="([^"]+)"', txt)
    name = mn.group(1) if mn else '(from macro)'
    mm = re.search(r'#define MAVLINK_MSG_ID_(\w+)\s+\d+', txt)
    macro = mm.group(1) if mm else 'NONE'
    flag = '' if name == macro or name == '(from macro)' else '  <<< DIFF'
    print('%-16s msg-name=%-16s macro=%s%s' % (os.path.basename(h), name, macro, flag))
