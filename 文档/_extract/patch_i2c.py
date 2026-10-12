# -*- coding: utf-8 -*-
# 补丁: 移除 BSP_I2C3_Init (custom_bus 已删, i2c.c/MX_I2C3_Init 已恢复)
import io, os, sys

B = r'E:\Embeded\RoboMaster\WUST2027\Aerial\G474 V1.0\Code'

def patch(rel, old, new):
    q = os.path.join(B, rel)
    with io.open(q, 'rb') as f:
        data = f.read()
    nl = b'\r\n' if b'\r\n' in data else b'\n'
    old_b = old.replace('\n', nl.decode('utf-8')).encode('utf-8')
    new_b = new.replace('\n', nl.decode('utf-8')).encode('utf-8')
    n = data.count(old_b)
    if n == 0:
        # 尝试仅 LF 版本(跨行块可能因 CRLF 匹配失败)
        old_lf = old.encode('utf-8')
        new_lf = new.encode('utf-8')
        n = data.count(old_lf)
        if n:
            data = data.replace(old_lf, new_lf)
            print('patched(LF) %s x%d' % (rel, n))
        else:
            print('!! NOT FOUND %s' % rel)
        with io.open(q, 'wb') as f:
            f.write(data)
        return
    data = data.replace(old_b, new_b)
    with io.open(q, 'wb') as f:
        f.write(data)
    print('patched %s x%d' % (rel, n))

# 1) main.c: 删除 BSP_I2C3_Init(); 整行 (MX_I2C3_Init 已在上方调用)
patch(r'Core\Src\main.c',
      '    BSP_I2C3_Init();\n',
      '')

# 2) VL53L1.c: VL53L1_Init 去掉 BSP_I2C3_Init
patch(r'User\Hardware\VL53L1.c',
      '    /* I2C3 已在 main.c 启动阶段初始化, 这里重复调用可确保就绪 */\n'
      '    BSP_I2C3_Init();\n'
      '\n'
      '    /* 写默认配置并完成一次自检 */\n'
      '    vl53l1_init();',
      '    /* I2C3 已由 main.c 的 MX_I2C3_Init() 初始化(i2c.c) */\n'
      '    vl53l1_init();')

# 3) VL53L1.h: 注释同步
patch(r'User\Hardware\VL53L1.h',
      '//VL53L1初始化(内部 BSP_I2C3_Init() + 写默认配置 + 一次自检等待(带超时), 耗时较长)',
      '//VL53L1初始化(写默认配置+一次自检, 耗时较长; I2C3由main.c的MX_I2C3_Init()提供)')

# 4) Sensor.c: 注释同步
patch(r'User\Task\Sensor.c',
      '    SPA06_Init();     // 内部 BSP_I2C3_Init()\n'
      '    VL53L1_Init();    // 内部 BSP_I2C3_Init(), 耗时较长',
      '    SPA06_Init();     // I2C3 由 main.c 的 MX_I2C3_Init() 初始化\n'
      '    VL53L1_Init();    // 写默认配置+一次自检, 耗时较长')

print('done')
