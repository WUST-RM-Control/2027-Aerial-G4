/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS__
#define __INS__

#include "main.h"

#define X 0
#define Y 1
#define Z 2

#define INS_TASK_PERIOD 1      /* 旧接口残留, 实际节拍见 Flight.h 的 FLIGHT_LOOP_HZ */

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];  // 角速度
    float Accel[3]; // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float Last_Yaw;
    float YawTotalAngle;
    float Yaw_Speed;    // 偏航角速度 (deg/s 即 °/s, 滤波后)
    float Pitch_Speed;  // 俯仰角速度 (deg/s 即 °/s, 滤波后, 与控制内环符号一致)
    float Roll_Speed;   // 横滚角速度 (deg/s 即 °/s, 滤波后, 与控制内环符号一致)
} INS_t;


/**
 * @brief 用于修正安装误差的参数,demo中可无视
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern INS_t INS;

void INS_Init(void);

/* Flight_Task 用接口: 初始化(只一次) / 单拍解算(4kHz 调用, 内部不延时) */
void INS_Init_All(void);
void INS_Update(void);

void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif
