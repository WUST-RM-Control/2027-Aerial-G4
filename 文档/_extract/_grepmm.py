import os
p = r'E:\Embeded\RoboMaster\WUST2027\Aerial\G474 V1.0\Code\Drivers\BSP\Components\vl53l1cb\modules\vl53l1_register_map.h'
for ln in open(p, encoding='utf-8', errors='replace'):
    s = ln.strip()
    if s.startswith('#define') and any(k in s for k in
        ['RANGE_MM', 'FINAL_CROSSTALK', 'RANGE_STATUS', 'RANGE_OFFSET', 'SIGNAL_RATE', 'AMBIENT_RATE', 'EFFECTIVE_SPADS']):
        print(s)
