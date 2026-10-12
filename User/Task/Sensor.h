#ifndef __Sensor__
#define __Sensor__

#include "main.h"

typedef struct
{
    //传感器数据
    float BatV;
    float Bar_Height_mm;
    float Bar_Height_Speed_mm;
    float TOF_Height_mm;
    float TOF_Height_Speed_mm;
    float Flow_Pix_Vx;   // 光流原始计数 X(向右为正, 已取反)
    float Flow_Pix_Vy;   // 光流原始计数 Y(向前为正)
    
    //融合数据
    float Height_mm;    // 融合滤波后的高度, 相对起飞点 (mm)
    float Height_Speed_mm; // 融合垂直速度 (mm/s, 向上为正)
    
    float Vel_X_mm;     // 机体速度 X (mm/s), 向右为正, 高度校准+陀螺补偿+滤波
    float Vel_Y_mm;     // 机体速度 Y (mm/s), 向前为正, 高度校准+陀螺补偿+滤波
} Sensor_StructTypedef;

extern Sensor_StructTypedef Sensor;

void Bar_Update(float *Bar_Height_Out, float *Bar_Height_Speed_Out, float Bar_Height, float Dt);

void TOF_Update(float *TOF_Height_Out, float *TOF_Height_Speed_Out, float TOF_Distence,
                      float Cos_Tilt, float Dt);


void Sensor_Flow_Update(float *Vx_out, float *Vy_out, 
                        float Pix_Vx, float Pix_Vy, float Squal,
                        float Height_mm, float Body_Gyro_X_rad, float Body_Gyro_Y_rad, 
                        float dt);

#endif
