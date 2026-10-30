#ifndef __Sensor__
#define __Sensor__

#include "main.h"

typedef struct
{
    //传感器数据
    float BatV;
    float Bar_Height_mm;        // 气压计相对高度(未融合, mm)
    float Bar_Height_Speed_mm;  // 气压高度微分(滤波后, mm/s)
    
    //融合数据(V3 无光流/TOF, 高度直接取气压计; 水平速度为 0)
    float Height_mm;        // 高度, 相对起飞点 (mm)
    float Height_Speed_mm;  // 垂直速度 (mm/s, 向上为正)
    
    float Vel_X_mm;     // 机体速度 X (mm/s), 向右为正 (V3 无光流观测, 恒 0)
    float Vel_Y_mm;     // 机体速度 Y (mm/s), 向前为正 (V3 无光流观测, 恒 0)
} Sensor_StructTypedef;

extern Sensor_StructTypedef Sensor;

void Bar_Update(float *Bar_Height_Out, float *Bar_Height_Speed_Out, float Bar_Height, float Dt);

#endif
