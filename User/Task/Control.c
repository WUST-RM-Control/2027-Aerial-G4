/*============================================================================
 * Control.c —— 飞行控制
 *
 *  摇杆 ─┬─ 姿态外环(P) ─┬─ 角速度内环(PID+FF) ─┬─ 混控 ─ DShot
 *        └─ 摇杆前馈      │                      │
 *  高度环(高度P -> 速度P) ─┴────── 总距 ──────────┘
 *
 *  模式(SB): -1 自稳 / 0 定高 / 1 定点; Acro(摇杆直接给角速度)保留可切
 *  调参: 全部在 User/Task/Parameter.h 第 5 节
 *  姿态数学(四元数/旋转矩阵/姿态误差): User/Algorithm/Attitude.c
 *===========================================================================*/
#include "Control.h"
#include "Parameter.h"
#include "Remote_Control.h"
#include "Sensor.h"
#include "INS.h"
#include "DShot.h"
#include "IMU_Cal.h"
#include "Attitude.h"

#include "math.h"

/* 功能开关与全部 PID 参数见 Parameter.h 第 5 节「飞行控制」 */

Control_StructTypedef Control;

static uint32_t Control_DWT_Count;
static float    Control_Dt;            /* 本拍控制周期(s) */

/*============================================================================
 *  输出层
 *===========================================================================*/
/* 占空比(0~100) -> DShot 油门值(只写帧缓冲, 由 DShot_Send 统一发一帧) */
void Control_Set_Duty(uint8_t Fan_ID, float Duty_100)
{
    Limit_float(&Duty_100, 95.0f, 0.0f);        /* 上限 90% 留余量 */
    if (Fan_ID < 4) Control.Duty[Fan_ID] = Duty_100;
    DShot_Set_Duty(Fan_ID, Duty_100);
}

/* 推力指令(0~100) -> 占空比: 推力正比于占空比², 反解后再乘电压补偿 */
void Control_Set_Thrust(uint8_t Fan_ID, float Thrust_100)
{
    Limit_float(&Thrust_100, 100.0f, 0.0f);
    Control_Set_Duty(Fan_ID, 100.0f * sqrtf(Thrust_100 / 100.0f) * Control.V_Comp);
}

/*============================================================================
 *  参数初始化 —— 调参只改这里
 *     PID_Init(结构体, Kp, Ki, Kd, Kf, I限幅, 输出限幅)
 *     调参顺序: 角速度内环 P -> D -> FF -> I, 再角度外环 P, 最后高度环
 *===========================================================================*/
static void Control_PID_Init(void)
{
    /* 高度环: 高度 -> 速度 -> 推力(悬停前馈见 Control_RateLoop) */
    PID_Init(&Control.PID_P_Height, CTRL_HEIGHT_P_KP, CTRL_HEIGHT_P_KI, CTRL_HEIGHT_P_KD,
             CTRL_HEIGHT_P_KF, CTRL_HEIGHT_P_IMAX, CTRL_HEIGHT_P_MAX);
    PID_Init(&Control.PID_S_Height, CTRL_HEIGHT_S_KP, CTRL_HEIGHT_S_KI, CTRL_HEIGHT_S_KD,
             CTRL_HEIGHT_S_KF, CTRL_HEIGHT_S_IMAX, CTRL_HEIGHT_S_MAX);

    /* 角度外环(deg 误差 -> deg/s 期望角速度), 只用 P; Roll/Pitch 对称共用一组 */
    PID_Init(&Control.PID_P_Roll, CTRL_ANGLE_RP_KP, CTRL_ANGLE_RP_KI, CTRL_ANGLE_RP_KD,
             CTRL_ANGLE_RP_KF, CTRL_ANGLE_RP_IMAX, CTRL_ANGLE_RP_MAX);
    PID_Init(&Control.PID_P_Pitch, CTRL_ANGLE_RP_KP, CTRL_ANGLE_RP_KI, CTRL_ANGLE_RP_KD,
             CTRL_ANGLE_RP_KF, CTRL_ANGLE_RP_IMAX, CTRL_ANGLE_RP_MAX);
    PID_Init(&Control.PID_P_Yaw, CTRL_ANGLE_YAW_KP, CTRL_ANGLE_YAW_KI, CTRL_ANGLE_YAW_KD,
             CTRL_ANGLE_YAW_KF, CTRL_ANGLE_YAW_IMAX, CTRL_ANGLE_YAW_MAX);

    /* 角速度内环(deg/s -> 推力差动), 线性区 = 40/0.15 约 267 deg/s; Roll/Pitch 共用 */
    PID_Init(&Control.PID_S_Roll, CTRL_RATE_RP_KP, CTRL_RATE_RP_KI, CTRL_RATE_RP_KD,
             CTRL_RATE_RP_KF, CTRL_RATE_RP_IMAX, CTRL_RATE_RP_MAX);
    PID_Init(&Control.PID_S_Pitch, CTRL_RATE_RP_KP, CTRL_RATE_RP_KI, CTRL_RATE_RP_KD,
             CTRL_RATE_RP_KF, CTRL_RATE_RP_IMAX, CTRL_RATE_RP_MAX);
    PID_Init(&Control.PID_S_Yaw, CTRL_RATE_YAW_KP, CTRL_RATE_YAW_KI, CTRL_RATE_YAW_KD,
             CTRL_RATE_YAW_KF, CTRL_RATE_YAW_IMAX, CTRL_RATE_YAW_MAX);
    PID_Set_D_LPF(&Control.PID_S_Roll,  CTRL_RATE_D_LPF_HZ);   /* D 项低通(Hz) */
    PID_Set_D_LPF(&Control.PID_S_Pitch, CTRL_RATE_D_LPF_HZ);
    PID_Set_D_LPF(&Control.PID_S_Yaw,   CTRL_RATE_D_LPF_HZ);

    /* 定点速度环(m/s -> 姿态角) */
    PID_Init(&Control.PID_S_X, CTRL_VEL_X_KP, CTRL_VEL_X_KI, CTRL_VEL_X_KD,
             CTRL_VEL_X_KF, CTRL_VEL_X_IMAX, CTRL_VEL_X_MAX);
    PID_Init(&Control.PID_S_Y, CTRL_VEL_Y_KP, CTRL_VEL_Y_KI, CTRL_VEL_Y_KD,
             CTRL_VEL_Y_KF, CTRL_VEL_Y_IMAX, CTRL_VEL_Y_MAX);
}

/*============================================================================
 *  输入层
 *===========================================================================*/
/* 解锁条件: 遥控在线 + SA 开关有效 + 本次上电陀螺零漂已实测 */
static void Control_UpdateEnable(void)
{
    Control.Gyro_Cal_OK = IMU_Cal_Gyro_Bias_Is_Valid();
    Control.Enable = (Remote.If_Remote_Connect && (Remote.SA != -1) && Control.Gyro_Cal_OK) ? 1u : 0u;
}

/* SB 三段开关 -> 控制模式 */
static void Control_UpdateMode(void)
{
    if      (Remote.SB == -1) Control.Control_Mode = Control_Mode_Level;
    else if (Remote.SB ==  0) Control.Control_Mode = Control_Mode_Level_Height;
    else if (Remote.SB ==  1) Control.Control_Mode = Control_Mode_Level_Point;
}

/* 摇杆 -> 油门/偏航/俯仰/横滚 (SC<0 日本手) */
static void Control_ReadStick(float *Thr, float *Yw, float *Pt, float *Rl)
{
    if (Remote.SC < 0)
    {
        *Thr = Remote.Right_Y;  *Yw = Remote.Right_X;
        *Pt  = Remote.Left_Y;   *Rl = Remote.Left_X;
    }
    else
    {
        *Thr = Remote.Left_Y;   *Yw = Remote.Left_X;
        *Pt  = Remote.Right_Y;  *Rl = Remote.Right_X;
    }
}

/* 反馈量 */
static void Control_UpdateFeedback(void)
{
    Control.Feedback_P_Height = Sensor.Height_mm;
    Control.Feedback_P_Roll   = INS.Roll;
    Control.Feedback_P_Pitch  = INS.Pitch;
    Control.Feedback_P_Yaw    = INS.YawTotalAngle;
    memcpy(Control.Feedback_Q, INS.q, sizeof(Control.Feedback_Q));

    Control.Feedback_S_Height = Sensor.Height_Speed_mm;
    Control.Feedback_S_Roll   = INS.Roll_Speed;
    Control.Feedback_S_Pitch  = INS.Pitch_Speed;
    Control.Feedback_S_Yaw    = INS.Yaw_Speed;

    Control.Feedback_V_X = Sensor.Vel_X_mm;
    Control.Feedback_V_Y = Sensor.Vel_Y_mm;
}

/*============================================================================
 *  杆量曲线
 *===========================================================================*/
/* 油门杆(-1~1) -> 推力域(0~100) */
static float Control_ThrottleCurve(float Thrust_Stick)
{
    float s = (Thrust_Stick + 1.0f) * 0.5f;     /* 杆量 -> 0~1 */
    Limit_float(&s, 1.0f, 0.0f);

#if (CONTROL_THROTTLE_SQUARE == 1u)
    return 100.0f * s * s;                      /* 杆量线性到占空比: duty = 100*s */
#else
    return 100.0f * s;                          /* 杆量线性到推力 */
#endif
}

/* Acro 杆量: 死区 + Expo(中段手感, 满杆仍为 1) */
static float Control_AcroCurve(float Stick)
{
    const float Expo = 0.0f;                    /* 0=线性, 0.3=中段更柔和 */

    Limit_float(&Stick, 1.0f, -1.0f);
    Deadline_Line_Float(&Stick, 0.02f);         /* 死区 + 重映射 */

    return (1.0f - Expo) * Stick + Expo * Stick * Stick * Stick;
}

/*============================================================================
 *  目标量
 *===========================================================================*/
static void Control_UpdateTarget(float Thr, float Yw, float Pt, float Rl)
{
    switch (Control.Control_Mode)
    {
    case Control_Mode_Acro:                     /* 摇杆直接给期望角速度, 无航向保持 */
        Control.Target_Height = Control_ThrottleCurve(Thr);
        Control.Rate_Stick[0] =   90.0f * Control_AcroCurve(Rl);   /* Roll  */
        Control.Rate_Stick[1] =  -90.0f * Control_AcroCurve(Pt);   /* Pitch */
        Control.Rate_Stick[2] = -120.0f * Control_AcroCurve(Yw);   /* Yaw   */
        Control.Target_Yaw = INS.YawTotalAngle;                    /* 切回角度模式不转头 */
        break;

    case Control_Mode_Level_Height:             /* 定高 */
        Control.Target_Height += Control_Dt * Remote.Right_Y * 250.0f;
        Limit_float(&Control.Target_Height,  Control.Feedback_P_Height + 250,  Control.Feedback_P_Height - 250);
        Control.Target_Pitch = -Pt * 60.0f;
        Control.Target_Roll  =  Rl * 60.0f;
        Control.Target_Yaw  -= Control_Dt * Yw * 90.0f;
        break;

    case Control_Mode_Level_Point:              /* 定点 */
        Control.Target_Height += Control_Dt * Remote.Right_Y * 1000.0f;
        Control.Target_V_X = Remote.Left_X * 200.0f;
        Control.Target_V_Y = Remote.Left_Y * 200.0f;
        Control.Target_Yaw -= Control_Dt * Remote.Right_X * 90.0f;
        break;

    default:                                    /* 自稳(Control_Mode_Level) */
        Control.Target_Height = Control_ThrottleCurve(Thr);
        Control.Target_Pitch = -Pt * 60.0f;
        Control.Target_Roll  =  Rl * 60.0f;
        Control.Target_Yaw  -= Control_Dt * Yw * 300.0f;
        break;
    }

    /* SA 最低位: 航向归零 + 定高到当前高度 +10cm */
    if (Remote.SA <= -0.5f)
    {
        Control.Target_Yaw = INS.YawTotalAngle;
        if (Control.Control_Mode != Control_Mode_Acro)
            Control.Target_Height = Sensor.Height_mm + 100.0f;
    }
}

/*============================================================================
 *  一键滚转: SD 上升沿起转, 用实测角速度积分判断进度
 *===========================================================================*/
#if CONTROL_AUTO_ROLL
static void Control_AutoRollUpdate(void)
{
    static uint8_t SD_Last = 1;                     /* 初值 1: 要求先松开, 防上电误触发 */
    uint8_t SD_Now = (Remote.SD > 0.5f) ? 1u : 0u;  /* SBUS 端点不是 ±1, 不能判 ==1 */

    if (Control.Auto_Roll_State == 1u)
    {
        Control.Auto_Roll_Time  += Control_Dt;
        Control.Auto_Roll_Angle += fabsf(Control.Feedback_S_Roll) * Control_Dt;

        if ((Control.Auto_Roll_Angle >= 360.0f) ||     /* 转够一圈 */
            (Control.Auto_Roll_Time  >  3.0f)    ||     /* 超时保护 */
            (!Control.Enable))                          /* 中途上锁 */
            Control.Auto_Roll_State = 0u;
    }
    else if (SD_Now && !SD_Last && Control.Enable && (INS.zn[2] > 0.90f))
    {
        Control.Auto_Roll_State = 1u;                   /* 需接近水平才允许起转 */
        Control.Auto_Roll_Angle = 0.0f;
        Control.Auto_Roll_Time  = 0.0f;
    }
    SD_Last = SD_Now;

    Control.Auto_Roll_Rate = 0.0f;
    if (Control.Auto_Roll_State == 1u)
    {
        float Scale = (360.0f - Control.Auto_Roll_Angle) / 180.0f;    /* 末 180deg 减速 */
        Limit_float(&Scale, 1.0f, 0.25f);
        Control.Auto_Roll_Rate = 360.0f * Scale;
    }
}
#endif

/*============================================================================
 *  位置环: 高度/速度 -> 总距与姿态目标
 *===========================================================================*/
static void Control_PositionLoop(void)
{
    if (Control.Control_Mode == Control_Mode_Acro)      /* 高度环不参与 */
    {
        PID_Clear_Output(&Control.PID_P_Height);
        PID_Clear_Output(&Control.PID_S_Height);
        return;
    }

    if (Control.Control_Mode == Control_Mode_Level_Point)   /* 速度环 -> 姿态目标 */
    {
        PID_Position_Calculate(&Control.PID_S_X, Control.Target_V_X, Control.Feedback_V_X);
        PID_Position_Calculate(&Control.PID_S_Y, Control.Target_V_Y, Control.Feedback_V_Y);
        Control.Target_Pitch = -Control.PID_S_Y.Output;
        Control.Target_Roll  =  Control.PID_S_X.Output;
    }

    PID_Position_Calculate(&Control.PID_P_Height, Control.Target_Height, Control.Feedback_P_Height);
}

/*============================================================================
 *  姿态外环: 姿态误差 -> 期望角速度(外环 P + 摇杆前馈)
 *===========================================================================*/
static void Control_AttitudeLoop(void)
{
    const float K_Ff = 0.0f;        /* 摇杆前馈系数: 0=不用, 0.3~1 打杆更跟手 */

    /* 目标四元数必须与陀螺同一套轴向(见 Attitude_QuatError 的说明):
     *   绕输出X = +Target_Roll, 绕输出Y = -Target_Pitch, 绕输出Z = +Target_Yaw
     * ★ 2026-10-19 换新 Frame_R 后 here 的 Roll/Pitch 参数位互换 */
    Attitude_Euler2Quat(Control.Target_Roll, -Control.Target_Pitch,
                        Control.Target_Yaw, Control.Target_Q);

#if (CONTROL_USE_QUATERNION == 1u)
    /* 四元数误差直接当目标, Feedback 填 0(等价 Target-Feedback) */
    Attitude_QuatError(Control.Feedback_Q, Control.Target_Q, Control.Err_Angle);
    PID_Position_Calculate(&Control.PID_P_Roll,  Control.Err_Angle[0], 0.0f);
    PID_Position_Calculate(&Control.PID_P_Pitch, Control.Err_Angle[1], 0.0f);
    PID_Position_Calculate(&Control.PID_P_Yaw,   Control.Err_Angle[2], 0.0f);
#else
    PID_Position_Calculate(&Control.PID_P_Roll,  Control.Target_Roll,  Control.Feedback_P_Roll);
    PID_Position_Calculate(&Control.PID_P_Pitch, Control.Target_Pitch, Control.Feedback_P_Pitch);
    PID_Position_Calculate(&Control.PID_P_Yaw,   Control.Target_Yaw,   Control.Feedback_P_Yaw);
#endif

    /* 摇杆前馈: 目标角变化率 x K_Ff; 目标被直接赋值造成跳变(如 SA 回中)该拍不前馈 */
    Control.Rate_FF[0] = 0.0f;
    Control.Rate_FF[1] = 0.0f;
    Control.Rate_FF[2] = 0.0f;
    if (Control_Dt > 1e-5f)
    {
        float dRoll  = Control.Target_Roll  - Control.Target_Last[0];
        float dPitch = Control.Target_Pitch - Control.Target_Last[1];
        float dYaw   = Control.Target_Yaw   - Control.Target_Last[2];

        if ((dRoll  <= 2.0f) && (dRoll  >= -2.0f)) Control.Rate_FF[0] = dRoll  / Control_Dt * K_Ff;
        if ((dPitch <= 2.0f) && (dPitch >= -2.0f)) Control.Rate_FF[1] = dPitch / Control_Dt * K_Ff;
        if ((dYaw   <= 2.0f) && (dYaw   >= -2.0f)) Control.Rate_FF[2] = dYaw   / Control_Dt * K_Ff;
    }
    Control.Target_Last[0] = Control.Target_Roll;
    Control.Target_Last[1] = Control.Target_Pitch;
    Control.Target_Last[2] = Control.Target_Yaw;

    /* 期望角速度: Acro 用摇杆直接值, 其余用外环 P + 前馈
     * (Acro 下外环仍在算, 只为切换模式时状态连续, 不被采用) */
    if (Control.Control_Mode == Control_Mode_Acro)
    {
        Control.Rate_SP[0] = Control.Rate_Stick[0];
        Control.Rate_SP[1] = Control.Rate_Stick[1];
        Control.Rate_SP[2] = Control.Rate_Stick[2];
    }
    else
    {
        Control.Rate_SP[0] = Control.PID_P_Roll.Output  + Control.Rate_FF[0];
        Control.Rate_SP[1] = Control.PID_P_Pitch.Output + Control.Rate_FF[1];
        Control.Rate_SP[2] = Control.PID_P_Yaw.Output   + Control.Rate_FF[2];
    }
    Limit_float(&Control.Rate_SP[0],  500.0f, -500.0f);
    Limit_float(&Control.Rate_SP[1],  500.0f, -500.0f);
    Limit_float(&Control.Rate_SP[2],  500.0f, -500.0f);

#if CONTROL_AUTO_ROLL
    if (Control.Auto_Roll_State == 1u) Control.Rate_SP[0] = Control.Auto_Roll_Rate;
#endif
}

/*============================================================================
 *  角速度内环 + 总距
 *===========================================================================*/
static void Control_RateLoop(void)
{
    PID_Position_Calculate(&Control.PID_S_Height, Control.PID_P_Height.Output, Control.Feedback_S_Height);
    PID_Position_Calculate(&Control.PID_S_Roll,   Control.Rate_SP[0], Control.Feedback_S_Roll);
    PID_Position_Calculate(&Control.PID_S_Pitch,  Control.Rate_SP[1], Control.Feedback_S_Pitch);
    PID_Position_Calculate(&Control.PID_S_Yaw,    Control.Rate_SP[2], Control.Feedback_S_Yaw);

    /* 总距: 自稳/Acro 直接跟油门杆, 定高/定点用高度速度环 + 悬停前馈 */
    if ((Control.Control_Mode == Control_Mode_Level) || (Control.Control_Mode == Control_Mode_Acro))
        Control.T_Height = Control.Target_Height;
    else
        Control.T_Height = 25.0f + Control.PID_S_Height.Output;    /* 15 = 悬停前馈 */
    Limit_float(&Control.T_Height, 100.0f, 0.0f);

    Control.T_Roll  = Control.PID_S_Roll.Output;
    Control.T_Pitch = Control.PID_S_Pitch.Output;
    Control.T_Yaw   = Control.PID_S_Yaw.Output;

#if CONTROL_TILT_COMP
    /* 倾斜补偿: 水平分量 = cos(倾角), 下限 0.7071(约 45deg) 防除零 */
    float cos_t = Attitude_GetCosTilt(Control.Feedback_Q);
    if (cos_t < 0.7071f) cos_t = 0.7071f;
    Control.T_Height /= cos_t;
    Limit_float(&Control.T_Height, 100.0f, 0.0f);
#endif
}

/*============================================================================
 *  混控: 十字(+)型 1=前臂CW 2=右臂CCW 3=左臂CCW 4=后臂CW
 *    T_Pitch>0 -> 前+后- (抬头)
 *    T_Roll >0 -> 左+右- (右滚)
 *    T_Yaw  >0 -> CW 桨加速 -> 机身反扭左偏
 *    (+)型关键: 前后臂只吃俯仰, 左右臂只吃横滚, 交叉项必须为 0
 *===========================================================================*/
static void Control_Mixer(void)
{
    float diff[4];
    float dmax, dmin, s = 1.0f, base, base_min, base_max;

    diff[0] =  Control.T_Pitch + Control.T_Yaw;
    diff[1] = -Control.T_Roll  - Control.T_Yaw;
    diff[2] =  Control.T_Roll  - Control.T_Yaw;
    diff[3] = -Control.T_Pitch + Control.T_Yaw;

    dmax = diff[0];
    dmin = diff[0];
    for (uint8_t i = 1; i < 4; i++)
    {
        if (diff[i] > dmax) dmax = diff[i];
        if (diff[i] < dmin) dmin = diff[i];
    }

    if ((dmax - dmin) > 100.0f) s = 100.0f / (dmax - dmin);   /* 差动超量程 -> 等比压缩 */

    /* 姿态优先: 差动全保留, 平移总距让四路都落进 [0,100] */
    base_min = -s * dmin;
    base_max = 100.0f - s * dmax;
    if (base_min < 0.0f)   base_min = 0.0f;
    if (base_max > 100.0f) base_max = 100.0f;
    if (base_max < base_min) base_max = base_min;

    base = Control.T_Height;
    Limit_float(&base, base_max, base_min);

    for (uint8_t i = 0; i < 4; i++)
    {
        Control.Thrust[i] = base + s * diff[i];
        if (Control.Enable && (Control.Thrust[i] < 3.0f))
            Control.Thrust[i] = 3.0f;                        /* 解锁后怠速, 防停转 */
        Limit_float(&Control.Thrust[i], 100.0f, 0.0f);
    }
}

/*============================================================================
 *  输出: 电压补偿 + 解锁判断
 *===========================================================================*/
static void Control_Output(void)
{
    Control.Vbat = Sensor.BatV;
#if CONTROL_V_COMP
    if (Control.Vbat > 5.5f)                     /* 低于此视为未接电池 */
    {
        Control.V_Comp = 7.4f / Control.Vbat;    /* 推力正比于 V², 掉压时补占空比 */
        Limit_float(&Control.V_Comp, 1.40f, 0.70f);
    }
    else
    {
        Control.V_Comp = 1.0f;
    }
#else
    Control.V_Comp = 1.0f;
#endif

    if (Control.Enable)
    {
        for (uint8_t i = 0; i < 4; i++) Control_Set_Thrust(i, Control.Thrust[i]);
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        Control.Thrust[i] = 0.0f;
        Control_Set_Duty(i, 0.0f);
    }
    PID_Clear_Output(&Control.PID_S_Height);
    PID_Clear_Output(&Control.PID_S_Roll);
    PID_Clear_Output(&Control.PID_S_Pitch);
    PID_Clear_Output(&Control.PID_S_Yaw);
}

/*============================================================================
 *  初始化(只跑一次, 由 Flight_Task 在 4kHz 节拍启动前调用)
 *===========================================================================*/
void Control_Init(void)
{
    Control.Target_Yaw = INS.YawTotalAngle;
    Control.V_Comp     = 1.0f;

    Control_PID_Init();
    DShot_Init();

    /* 预置 DWT 计时起点: 否则第一拍 dt 会是"上电到现在"的一个大值 */
    DWT_GetDeltaT(&Control_DWT_Count);
}

/*============================================================================
 *  单拍控制(4kHz = 250us, 由 Flight_Task 在 TIM6 节拍里调用, 内部不延时)
 *    DShot 也跟着每拍发一帧 -> 输出 4kHz
 *===========================================================================*/
void Control_Update(void)
{
    float Thr, Yw, Pt, Rl;

    Control_Dt = DWT_GetDeltaT(&Control_DWT_Count);

    Control_UpdateEnable();
    Control_UpdateMode();
    Control_ReadStick(&Thr, &Yw, &Pt, &Rl);
    Control_UpdateTarget(Thr, Yw, Pt, Rl);
    Control_UpdateFeedback();

#if CONTROL_AUTO_ROLL
    Control_AutoRollUpdate();
#endif

    Control_PositionLoop();
    Control_AttitudeLoop();
    Control_RateLoop();
    Control_Mixer();
    Control_Output();

    DShot_Send();       /* 4 路同帧, 电调靠连续收帧判断信号正常 */
}
