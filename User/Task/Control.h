#ifndef __Control_H__
#define __Control_H__

#include "main.h"

typedef enum
{
    Control_Mode_Level = 0,     // 自稳: 摇杆 -> 目标角度
    Control_Mode_Level_Height,  // 定高: + 高度环
    Control_Mode_Level_Point,   // 定点: + 高度环 + 速度环
    Control_Mode_Acro           // Acro: 摇杆直接 -> 期望角速度
} Control_Mode_EnumTypedef;

typedef struct
{
    uint8_t Enable;         // 1=已解锁
    uint8_t Gyro_Cal_OK;    // 1=本次上电陀螺零漂实测通过
    uint8_t Control_Mode;

    float Target_Height;    // 推力域 0~100 (自稳) / 高度 mm (定高)
    float Target_Pitch;     // deg
    float Target_Roll;      // deg
    float Target_Yaw;       // deg, 累计总角度

    /* 姿态外环 */
    float Target_Q[4];      // 目标四元数 (w,x,y,z)
    float Feedback_Q[4];    // 当前四元数 (取自 INS.q)
    float Err_Angle[3];     // 姿态误差 deg: [0]=Roll [1]=Pitch [2]=Yaw

    /* 角速度环路 */
    float Rate_SP[3];       // 期望角速度 °/s = 外环 P + 摇杆前馈
    float Rate_FF[3];       // 摇杆前馈项
    float Rate_Stick[3];    // Acro: 摇杆直接给出的期望角速度
    float Target_Last[3];   // 上一拍目标角, 用于求前馈

    float Feedback_P_Height;
    float Feedback_P_Pitch;
    float Feedback_P_Roll;
    float Feedback_P_Yaw;

    float Feedback_S_Height;
    float Feedback_S_Pitch;
    float Feedback_S_Roll;
    float Feedback_S_Yaw;

    PID_Struct_TypeDef PID_P_Height;
    PID_Struct_TypeDef PID_P_Pitch;
    PID_Struct_TypeDef PID_P_Roll;
    PID_Struct_TypeDef PID_P_Yaw;

    PID_Struct_TypeDef PID_S_Height;
    PID_Struct_TypeDef PID_S_Pitch;
    PID_Struct_TypeDef PID_S_Roll;
    PID_Struct_TypeDef PID_S_Yaw;

    /* 定点速度环 */
    float Target_V_X;
    float Target_V_Y;
    float Feedback_V_X;
    float Feedback_V_Y;

    PID_Struct_TypeDef PID_S_X;
    PID_Struct_TypeDef PID_S_Y;

    /* 三轴差动指令(推力域) */
    float T_Height;
    float T_Pitch;
    float T_Roll;
    float T_Yaw;

    /* 一键滚转 */
    uint8_t Auto_Roll_State;    // 0=空闲 1=滚转中
    float   Auto_Roll_Rate;     // 当前期望角速度 °/s
    float   Auto_Roll_Angle;    // 已转角度 deg
    float   Auto_Roll_Time;     // 已用时间 s

    /* 输出 */
    float Vbat;
    float V_Comp;           // 电压补偿系数
    float Thrust[4];        // 混控后推力 0~100, 0~3 = 1~4号电机 -> TIM4 CH1~CH4
                            // 十字(+)型: 0=前臂(CW) 1=右臂(CCW) 2=左臂(CCW) 3=后臂(CW)
    float Duty[4];          // 最终占空比 0~100 (遥测用)
} Control_StructTypedef;

extern Control_StructTypedef Control;



void Control_Set_Duty(uint8_t Fan_ID, float Duty_100);
void Control_Set_Thrust(uint8_t Fan_ID, float Thrust_100);

/* Flight_Task 用接口: 初始化(PID+DShot) / 单拍控制(4kHz 调用, 内部不延时) */
void Control_Init(void);
void Control_Update(void);


#endif
