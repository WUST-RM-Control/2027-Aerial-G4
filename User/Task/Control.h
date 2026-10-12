#ifndef __Control_H__
#define __Control_H__

#include "main.h"

typedef enum
{
    Control_Mode_Level = 0,
    Control_Mode_Level_Height,
    Control_Mode_Level_Point,
    Control_Mode_Acro          // Acro(手动率): 摇杆直接给期望角速度, 姿态外环不参与
} Control_Mode_EnumTypedef;

typedef struct
{
    uint8_t Enable;
    uint8_t Gyro_Cal_OK;    // 1=本次上电陀螺零漂实测成功; 0=未标定好(禁止解锁)
    uint8_t Control_Mode;   // 0=手动, 1=定高

    float Target_Height;
    float Target_Pitch;
    float Target_Roll;
    float Target_Yaw;
    
    /*===| 姿态外环: 四元数误差 / 欧拉角(两种都保留) |===*/
    float Target_Q[4];      // 目标姿态四元数(w,x,y,z, 与 INS.q 同约定: 机体系->导航系)
    float Feedback_Q[4];    // 当前姿态四元数(直接取自 INS.q)
    float Err_Angle[3];     // 姿态误差(机体轴旋转矢量, deg): [0]=Roll [1]=Pitch [2]=Yaw

    /*===| A 方案: 期望角速度, 摇杆前馈与摇杆杆量 |===*/
    float Rate_SP[3];       // 角速度内环期望值(°/s) = 角度外环 P + 摇杆前馈
    float Rate_FF[3];       // 摇杆前馈项(°/s), 便于调参/调试
    float Rate_Stick[3];    // Acro 模式: 摇杆直接给出的期望角速度(°/s)
    float Target_Last[3];   // 上一拍目标姿态角(°), 用于求摇杆前馈
    
    float Feedback_P_Height;
    float Feedback_P_Pitch; // 姿态反馈: INS.Pitch
    float Feedback_P_Roll;  // 姿态反馈: INS.Roll
    float Feedback_P_Yaw;   // 姿态反馈: INS.YawTotalAngle(累计总角度)
    
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
    
    float Target_V_X;
    float Target_V_Y;
    float Feedback_V_X;
    float Feedback_V_Y;

    PID_Struct_TypeDef PID_S_X;
    PID_Struct_TypeDef PID_S_Y;
    
    float T_Height;
    float T_Pitch;
    float T_Roll;
    float T_Yaw;
    
    /*===| 一键滚转(SD 按下触发) |===*/
    uint8_t Auto_Roll_State;   // 0=空闲 1=滚转中
    float   Auto_Roll_Rate;    // 当前滚转期望角速度(°/s, 滚转中覆盖姿态外环输出)
    float   Auto_Roll_Angle;   // 已滚转角度(°, 由角速度积分得到)
    float   Auto_Roll_Time;    // 已进行时间(s)

    /*===| 推力域(归一化推力指令 0~100)相关 |===*/
    float Vbat;           // 电池电压(V), 取自 Sensor.BatV
    float V_Comp;         // 电压补偿系数(推力->占空比 前乘)
    float Thrust[4];      // 混控后的推力指令(0~100, 归一化推力域)
                          // 下标 0~3 = 1~4号电机 -> TIM4 CH1~CH4
                          // 十字(+)型机架方位: 0=左臂 1=后臂 2=右臂 3=前臂
    float Duty[4];        // 最终占空比(0~100, 已含曲线反解与电压补偿)
} Control_StructTypedef;

extern Control_StructTypedef Control;



void Control_Set_Duty(uint8_t Fan_ID, float Duty_100);
void Control_Set_Thrust(uint8_t Fan_ID, float Thrust_100);


#endif
