#ifndef __PID__
#define __PID__

#include "main.h"


/*===| PID结构体定义 |===*/
typedef struct
{
    uint32_t DWT_Count;
    
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    
    float Target;
    float Feedback;
    
    float Error;
    float Error_Last;      
    float Feedback_Last;    /* 上一拍测量值(D 项对测量微分用, 不跟目标跑) */

    float K_Output;
    float I_Output;
    float D_Output;
    float F_Output;
    
    float Output;
    float Output_Unsat;     /* 限幅前的总和(调试用: 看饱和程度) */
    
    float I_Output_Max;     /* I 项限幅; =0 表示不用积分(直接关闭 I) */
    float Output_Max;

    float Dt_Nominal;       /* 标称控制周期(s): dt 异常时兜底, 避免除零 */
    float Dt;               /* 本拍实际周期(s), 只读 */
    float D_LPF_Fc;         /* D 项一阶低通截止频率(Hz); =0 关闭 D */
    float D_LPF_Alpha;      /* 本拍滤波系数(只读) */
    uint8_t First_Run;      /* 1=还没跑过: 首拍不输出 D 尖峰 */
} PID_Struct_TypeDef;

/*===| PID参数初始化(参数顺序保持不变, 兼容旧调用) |===*/
void PID_Init(PID_Struct_TypeDef *PID_Struct, float Kp, float Ki, float Kd, float Kf, float I_Output_Max, float Output_Max);
/*===| PID位置式计算 |===*/
void PID_Position_Calculate(PID_Struct_TypeDef *PID_Struct, float Target, float Feedback);
/*===| PID清空积分与输出 |===*/
void PID_Clear_Output(PID_Struct_TypeDef *PID_Struct);

/*===| 附加配置(可选, 不调则用 PID_Init 里的默认值) |===*/
void PID_Set_D_LPF(PID_Struct_TypeDef *PID_Struct, float Fc_Hz);   /* D 项低通截止频率, 0=关闭 D */
void PID_Set_Dt(PID_Struct_TypeDef *PID_Struct, float Dt);         /* 固定控制周期时显式告诉 PID */



#endif
