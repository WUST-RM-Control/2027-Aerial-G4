#include "PID.h"
#include "DWT.h"

#define PID_DT_DEFAULT      0.00025f /* 默认标称控制周期(s): 4kHz(250us), 仅作 dt 异常时的兜底 */
#define PID_D_LPF_DEFAULT   100.0f   /* D 项低通默认截止频率(Hz), 0=关闭 D */

#ifndef PI
#define PI                  3.14159265358979f
#endif

/*===| PID参数初始化(参数顺序不变) |===*/
void PID_Init(PID_Struct_TypeDef *PID_Struct, float Kp, float Ki, float Kd, float Kf, float I_Output_Max, float Output_Max)
{
    PID_Struct->Kp = Kp;
    PID_Struct->Ki = Ki;
    PID_Struct->Kd = Kd;
    PID_Struct->Kf = Kf;
    PID_Struct->I_Output_Max = I_Output_Max;
    PID_Struct->Output_Max = Output_Max;

    /* 运行时状态全部归零, 允许反复初始化 */
    PID_Struct->Target = 0.0f;
    PID_Struct->Feedback = 0.0f;
    PID_Struct->Error = 0.0f;
    PID_Struct->Error_Last = 0.0f;
    PID_Struct->Feedback_Last = 0.0f;
    PID_Struct->K_Output = 0.0f;
    PID_Struct->I_Output = 0.0f;
    PID_Struct->D_Output = 0.0f;
    PID_Struct->F_Output = 0.0f;
    PID_Struct->Output = 0.0f;
    PID_Struct->Output_Unsat = 0.0f;
    PID_Struct->Dt_Nominal = PID_DT_DEFAULT;
    PID_Struct->Dt = PID_DT_DEFAULT;
    PID_Struct->D_LPF_Fc = PID_D_LPF_DEFAULT;
    PID_Struct->D_LPF_Alpha = 0.0f;
    PID_Struct->First_Run = 1u;
}

/*===| 附加配置: D 项低通截止频率(Hz), 0 表示不用 D |===*/
void PID_Set_D_LPF(PID_Struct_TypeDef *PID_Struct, float Fc_Hz)
{
    PID_Struct->D_LPF_Fc = (Fc_Hz > 0.0f) ? Fc_Hz : 0.0f;
}

/*===| 附加配置: 固定控制周期时直接告诉 PID(否则用 DWT 实测 dt) |===*/
void PID_Set_Dt(PID_Struct_TypeDef *PID_Struct, float Dt)
{
    if (Dt > 1e-5f) PID_Struct->Dt_Nominal = Dt;
}

/*===
 * PID位置式计算(A 方案: 标准多旋翼环路用)
 *
 *     u = Kp*e  +  Ki*∫e dt  -  Kd*d(测量)/dt  +  Kf*目标
 *
 * 相比旧实现改了 5 处:
 *  1) D 项对“测量值”微分(而不是误差): 打杆时目标突变不再产生 D 尖峰,
 *     所以 Kd 可以放心调大
 *  2) D 项一阶低通(Fc = D_LPF_Fc): 抑制微分噪声, 否则 Kd 一上去就抖
 *  3) I 项: I_Output_Max=0 视为不用积分; 输出饱和时做 back-calculation
 *     (把 I 拉回到刚好不饱和), 防积分饱和
 *  4) dt 兜底: 首拍调用(DWT_Count=0 -> dt 很大)或任务被拖慢时用标称
 *     周期, 再也不会出现 dt=0 除零出 Inf/NaN
 *  5) NaN 自检: 数值一旦异常就整体复位, 避免 NaN 顺着 Error_Last/I 一直
 *     传下去把整条环路永久“污染”(表现就是某轴突然不响应/电机停转)
 *===*/
void PID_Position_Calculate(PID_Struct_TypeDef *PID_Struct, float Target, float Feedback)
{
    float Dt = DWT_GetDeltaT(&PID_Struct->DWT_Count);
    float D_Meas, W, Unsat;
    /* 是否启用积分: Ki 与 I_Output_Max 都要有效(任一为 0 都视为不用 I) */
    uint8_t I_On = ((PID_Struct->I_Output_Max > 0.0f) && (PID_Struct->Ki != 0.0f)) ? 1u : 0u;

    /*===| dt 兜底: 异常周期一律用标称值(绝不可能是 0) |===*/
    if ((Dt <= 1e-5f) || (Dt > 0.05f)) Dt = PID_Struct->Dt_Nominal;
    PID_Struct->Dt = Dt;

    /*===| 赋值与误差 |===*/
    PID_Struct->Target = Target;
    PID_Struct->Feedback = Feedback;
    PID_Struct->Error = Target - Feedback;

    /*===| P |===*/
    PID_Struct->K_Output = PID_Struct->Kp * PID_Struct->Error;

    /*===| I: 累积 + 限幅(不用 I 时恒为 0) |===*/
    if (I_On)
    {
        PID_Struct->I_Output += PID_Struct->Ki * PID_Struct->Error * Dt;
        if      (PID_Struct->I_Output >  PID_Struct->I_Output_Max) PID_Struct->I_Output =  PID_Struct->I_Output_Max;
        else if (PID_Struct->I_Output < -PID_Struct->I_Output_Max) PID_Struct->I_Output = -PID_Struct->I_Output_Max;
    }
    else
    {
        PID_Struct->I_Output = 0.0f;
    }

    /*===| D: 对测量值微分(首拍不产生 D) + 一阶低通 |===*/
    if (PID_Struct->First_Run)
    {
        PID_Struct->Feedback_Last = Feedback;
        PID_Struct->First_Run = 0u;
    }
    D_Meas = -(Feedback - PID_Struct->Feedback_Last) / Dt;          /* 测量变化率的负值 */
    W = 2.0f * PI * PID_Struct->D_LPF_Fc * Dt;                      /* 一阶低通系数 */
    PID_Struct->D_LPF_Alpha = (W > 0.0f) ? (W / (1.0f + W)) : 0.0f;
    PID_Struct->D_Output += PID_Struct->D_LPF_Alpha * (PID_Struct->Kd * D_Meas - PID_Struct->D_Output);

    /*===| 前馈 |===*/
    PID_Struct->F_Output = PID_Struct->Kf * PID_Struct->Target;

    /*===| 求和 -> 限幅 -> 饱和回退(防积分饱和) |===*/
    Unsat = PID_Struct->K_Output + PID_Struct->I_Output + PID_Struct->D_Output + PID_Struct->F_Output;
    PID_Struct->Output_Unsat = Unsat;

    if (Unsat > PID_Struct->Output_Max)
    {
        PID_Struct->Output = PID_Struct->Output_Max;
        if (I_On)                                                   /* back-calculation: 把 I 拉回 */
        {
            PID_Struct->I_Output -= (Unsat - PID_Struct->Output_Max);
            if (PID_Struct->I_Output < -PID_Struct->I_Output_Max) PID_Struct->I_Output = -PID_Struct->I_Output_Max;
        }
    }
    else if (Unsat < -PID_Struct->Output_Max)
    {
        PID_Struct->Output = -PID_Struct->Output_Max;
        if (I_On)
        {
            PID_Struct->I_Output -= (Unsat + PID_Struct->Output_Max);
            if (PID_Struct->I_Output >  PID_Struct->I_Output_Max) PID_Struct->I_Output =  PID_Struct->I_Output_Max;
        }
    }
    else
    {
        PID_Struct->Output = Unsat;
    }

    /*===| 记录 |===*/
    PID_Struct->Error_Last = PID_Struct->Error;
    PID_Struct->Feedback_Last = Feedback;

    /*===| NaN 自检: 数值异常就整体复位, 防止“永久污染” |===*/
    if ((PID_Struct->Output   != PID_Struct->Output) ||
        (PID_Struct->I_Output != PID_Struct->I_Output) ||
        (PID_Struct->D_Output != PID_Struct->D_Output))
    {
        PID_Struct->I_Output = 0.0f;
        PID_Struct->D_Output = 0.0f;
        PID_Struct->Output = 0.0f;
        PID_Struct->Output_Unsat = 0.0f;
        PID_Struct->Error_Last = 0.0f;
        PID_Struct->Feedback_Last = Feedback;
        PID_Struct->First_Run = 1u;
    }
}

/*===| PID清空积分与输出(保留测量值历史, 重新使能时不会抽一下) |===*/
void PID_Clear_Output(PID_Struct_TypeDef *PID_Struct)
{
    PID_Struct->I_Output = 0;
    PID_Struct->D_Output = 0;
    PID_Struct->Output = 0;
    PID_Struct->Output_Unsat = 0;
}



