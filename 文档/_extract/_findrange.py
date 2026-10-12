import os, glob
base = r'E:\Embeded\RoboMaster\WUST2027\Aerial\G474 V1.0\Code\Drivers\BSP\Components\vl53l1cb\modules'
keys = ['RANGE_MM', 'CROSSTALK_CORRECTED_RANGE', 'FINAL_RANGE', 'FINAL_CROSSTALK',
        'RANGE_STATUS', 'STREAM_COUNT', 'INTERRUPT_STATUS', 'MEASUREMENT_READY',
        'PEAK_SIGNAL', 'AMBIENT', 'EFFECTIVE_SPAD']
for f in glob.glob(os.path.join(base, 'vl53l1_register_map.h')):
    for ln in open(f, encoding='utf-8', errors='replace'):
        s = ln.strip()
        if s.startswith('#define') and any(k in s for k in keys):
            if 'PREV' in s or 'SHADOW' in s:
                continue
            print(s)
