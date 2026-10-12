# -*- coding: utf-8 -*-
import io

base = r'E:\Embeded\RoboMaster\WUST2027\Aerial\G474 V1.0\Code'

def patch(path, pred, insert):
    with io.open(path, encoding='utf-8', newline='') as f:
        lines = f.readlines()
    if any(insert.strip() in l for l in lines):
        print('skip (already)', path, insert.strip())
        return
    for i, l in enumerate(lines):
        if pred(l):
            lines.insert(i + 1, insert)
            with io.open(path, 'w', encoding='utf-8', newline='') as f:
                f.writelines(lines)
            print('patched', path, 'after line', i + 1)
            return
    print('NOT FOUND', path)

# main.h: 在 dma.h 之后插入 custom_bus.h
patch(base + r'\Core\Inc\main.h',
      lambda l: l.startswith('#include "dma.h"'),
      '#include "custom_bus.h"\r\n')

# main.c: 在 DWT_Init(170) 之后插入 BSP_I2C3_Init()
patch(base + r'\Core\Src\main.c',
      lambda l: l.find('DWT_Init(170)') >= 0,
      '    BSP_I2C3_Init();\r\n')
