# -*- coding: utf-8 -*-
import inspect, re
from pymavlink.generator import mavparse
src = inspect.getsource(mavparse)
i = src.find('def ordered_fields')
if i == -1:
    i = src.find('ordered_fields')
    # 打印所有出现处
    for m in re.finditer('ordered_fields', src):
        print('--- at', m.start())
        print(src[m.start()-200:m.start()+300])
else:
    print(src[i:i+800])
