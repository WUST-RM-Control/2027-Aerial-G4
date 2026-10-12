#include "Control.h"
#include "Remote_Control.h"
#include "Sensor.h"
#include "INS.h"
#include "DShot.h"
#include "IMU_Cal.h"

#include "math.h"   // sinf/cosf/sqrtf/atan2f (四元数姿态误差 & 倾斜补偿)

#define Duty_Max 90.0f
//测试
#define CONTROL_TILT_COMPENSATION 1         /* 倾斜油门补偿 */
#define CONTROL_MIN_COS_TILT      0.7071f   /* 补偿上限: 等效约45°, 防除0 */


#define CONTROL_V_COMP_ENABLE  1             /* 电压补偿: duty *= V_REF / Vbat  (推力∝V², 掉压时自动补占空比) */
#define CONTROL_V_REF          7.4f         /* ★按实际电池改: 1S=3.7~4.2, 2S=7.4~8.4 */
#define CONTROL_V_COMP_MAX     1.40f        /* 补偿上限, 防低压时过冲 */
#define CONTROL_V_COMP_MIN     0.70f        
#define CONTROL_VBAT_MIN       3.0f         /* 低于此认为电压无效(未接电池), 不补偿 */
        
#define Hover_Thrust           50.0f        /* 悬停油门 */
#define CONTROL_MOTOR_IDLE     3.0f         /* 解锁后单电机最低推力指令(%), 防空中停转 */  /* 补偿上限: 等效约45°, 防除0 */

/* 油门杆(-1~1) -> 推力域指令(0~100) 的曲线选择:
 *   1 = 杆量线性映射到 duty(推荐): T = 100*s²  -> duty = 10√T = 100*s
 *       即“推多少杆给多少油门”, 手感同普通 PWM 油门; 推杆到 -0.9 只有 5% duty,
 *       地面小油起非常可控(悬停 T=15 对应杆位 -0.23)
 *   0 = 杆量线性映射到推力(原行为): T = 100*s -> 中位就有 70% duty,
 *       地面起飞“轻轻一推就跳”, 很容易一推就翻 */
#define CONTROL_THROTTLE_CURVE      1u

/*==============================================================================
 *  控制环路参数(A 方案: 三环级联 + 前馈 + 滤波)
 *
 *  摇杆 ─┬─> 姿态外环(姿态误差, 纯 P) ─┬─> 角速度内环(P+I+D+FF) ─> 推力差动 ─┐
 *        └─> 摇杆前馈(角度变化率xFF) ─┘                                    │
 *                    高度环(高度P -> 速度P) ────────> 总距 ──────> 混控 ─┴─> DShot
 *
 *  模式(SB 开关):
 *    SB=-1 -> 自稳 : 角度自稳(摇杆给目标姿态角 ±45°), 油门直接给推力, 高度环不参与
 *    SB= 0 -> 定高 : 角度自稳 + 高度环
 *    SB= 1 -> 定点 : 角度自稳 + 高度环 + 速度环
 *  (Acro 手动率模式的代码仍保留在下面, 想切回手动率只需把模式选择里
 *   的 Control_Mode_Level 改成 Control_Mode_Acro 即可)
 *
 *  调参顺序: 1) 角速度内环: 先 P(到刚不振荡) -> 再加 D(压过冲) -> 再加 FF(跟踪更准)
 *            2) 角度外环 P(从 8~10 起, 大就变“回中很冲”)
 *            3) 高度环: 先速度内环 P, 再高度外环 P
 *  调试建议: 把 PID_S_*.Target(期望角速度) 与 .Feedback(实测角速度) 上传到
 *            Vofa 看阶跃响应(超调/上升时间/振荡), 比凭手感快得多
 *============================================================================*/

/*--- 角度外环(角度 -> 期望角速度): 只用 P + 摇杆前馈 ---*/
#define CONTROL_ANGLE_KP_RP          40.0f   /* 每 1° 误差给多少 °/s (Roll/Pitch)          */
#define CONTROL_ANGLE_KP_YAW         20.0f   /* 每 1° 误差给多少 °/s (Yaw)                */
#define CONTROL_ANGLE_FF             0.0f   /* 摇杆前馈: 目标角度变化率 -> 期望角速度的比例
                                             * 0=不用前馈(全靠 P 追), 1=完全由摇杆速度决定;
                                             * 越大“打杆越跟手”, 推荐 0.3~1.0          */
#define CONTROL_ANGLE_FF_JUMP        2.0f   /* 单拍目标变化超过此值(°)视为跳变(如 SA 回中重置
                                             * 航向), 该拍不做前馈, 防巨大尖峰          */
#define CONTROL_RATE_SP_MAX        500.0f   /* 期望角速度限幅(°/s)                     */

/*--- Acro(手动率)模式: 摇杆直接给期望角速度, 姿态外环完全不参与 ---
 * ⚠ 飞之前先确认内环不饱和: 内环线性区 = OUTPUT_MAX / KP
 *   若 CONTROL_RATE_KP=0.2 / OUT_MAX=50 -> 线性区仅 250°/s, 而满杆就要 180°/s,
 *   误差一大就削顶, 表现为“打杆像开关、抽/抖”。
 *   建议先把 CONTROL_RATE_KP 降到 0.06~0.08、CONTROL_RATE_OUT_MAX 压到 25~30,
 *   CONTROL_RATE_I_MAX 提到 10~15, 再按 P->D->FF->I 重调。
 * ⚠ Acro 不会自稳(松杆会保持姿态), 需要自己稳住。 */
#define CONTROL_ACRO_RATE_RP       90.0f   /* 满杆期望角速度(°/s), Roll/Pitch      */
#define CONTROL_ACRO_RATE_YAW      120.0f   /* 满杆期望角速度(°/s), Yaw             */
#define CONTROL_ACRO_EXPO             0.0f  /* 摇杆曲线: 0=线性, 0.3=中间段更柔和   */
#define CONTROL_ACRO_DEADBAND        0.02f  /* 摇杆死区(归一化), 抑制回中抖动        */

/*--- 角速度内环(角速度 -> 推力差动, 推力域): P + I + D(测速微分) + 速率前馈 ---
 * 线性区(不削顶的最大率误差) = CONTROL_RATE_OUT_MAX / CONTROL_RATE_KP
 *   = 30/0.10 = 300°/s, 足够覆盖期望角速度上限 150°/s(约 2 倍)。
 * ⚠ KP/KI 太小会“看着方向对但拉不回来”: 地面推油门时重力/CG 偏心/电机拉力不均
 *   需要几到十几个推力单位的**常值差动**去顶, 而 P 只靠“率误差”产生,
 *   静态倾倒时率误差≈0 -> P 给不出东西; 这时全靠 I 慢慢积 -> KI 太小就永远顶不住。
 *   所以 KI 必须让 I 能在 1~3 秒内积到需要的工作点(I_MAX 同时给够上限)。 */
#define CONTROL_RATE_KP              0.1f  /* 先单独调到“刚好不振荡”                  */
#define CONTROL_RATE_KD             0.002f  /* 测速微分阻尼(已含一阶低通), 压 P 带来的过冲  */
#define CONTROL_RATE_KF             0.0f  /* 速率前馈(≈0.3×Kp): 减轻 I 负担、跟踪更准   */
#define CONTROL_RATE_KI             0.01f  /* 抗扰/配平主力(CG 偏/电机不均/风); 太小→地面顶不住  */
#define CONTROL_RATE_I_MAX          10.0f  /* I 项限幅(推力域), 防长时间累积            */
#define CONTROL_RATE_OUT_MAX        40.0f  /* 内环输出限幅(推力域)                    */
#define CONTROL_RATE_D_LPF_HZ       50.0f  /* D 项低通截止频率(Hz), 越低越钝但越抗噪     */

#define CONTROL_RATE_KP_YAW         0.15f  /* 偏航也要真控制: 太小则推油门时反作用力矩直接带着机身转 */
#define CONTROL_RATE_KD_YAW         0.001f
#define CONTROL_RATE_KF_YAW         0.0f
#define CONTROL_RATE_KI_YAW         0.0f
#define CONTROL_YAW_I_MAX            5.0f

/*--- 高度环 ---*/
#define CONTROL_HEIGHT_KP            0.8f
#define CONTROL_HEIGHT_KD           0.05f
#define CONTROL_HEIGHT_OUT_MAX     300.0f

#define CONTROL_HEIGHT_SPEED_KP      0.02f
#define CONTROL_HEIGHT_SPEED_KI     0.003f    
#define CONTROL_HEIGHT_SPEED_I_MAX  10.0f
#define CONTROL_HEIGHT_SPEED_OUT_MAX 30.0f

/*--- 定点模式速度环 ---*/
#define CONTROL_VEL_KP               0.03f
#define CONTROL_VEL_KI               0.05f
#define CONTROL_VEL_I_MAX           50.0f
#define CONTROL_VEL_OUT_MAX        100.0f

/*===| 一键滚转: SD 按下(通道8 置高) -> 绕机体X轴自动滚转一圈 |===*/
#define CONTROL_AUTO_ROLL_ENABLE    0           /* 1=启用 */
#define CONTROL_AUTO_ROLL_DIR       1.0f        /* 方向: +1=与右打杆同向, -1=反向 */
#define CONTROL_AUTO_ROLL_RATE      360.0f      /* 滚转期望角速度(°/s) */
#define CONTROL_AUTO_ROLL_ANGLE     360.0f      /* 滚转角度(°), 想转两圈改720 */
#define CONTROL_AUTO_ROLL_TAPER     180.0f       /* 末段减速区间(°), 保证停在整圈附近 */
#define CONTROL_AUTO_ROLL_MIN_SCALE 0.25f       /* 末段最低角速度比例 */
#define CONTROL_AUTO_ROLL_TIMEOUT   3.0f        /* 超时保护(s) */
#define CONTROL_AUTO_ROLL_START_COS 0.90f       /* 起转条件: cos(倾角)>此值(≈25°内才允许起转) */

/*===| 姿态外环: 选择"四元数误差"还是"欧拉角直减" |===
 * 1 = 四元数误差(Target_Q/Feedback_Q -> Err_Angle): 无 ±90° 奇异, 大角度/翻滚连续
 * 0 = 欧拉角(Target_xxx vs INS.Roll/Pitch/YawTotalAngle): 与旧版一致
 * 两者小角度下数值等价、符号约定相同, 切换不需要改遥控方向或 PID 符号 */
#define CONTROL_ATTITUDE_QUATERNION  1u

Control_StructTypedef Control;

uint32_t Control_DWT_Couter;
float Control_Dt;

/*==============================================================================
 *  姿态外环: 两种误差计算方式(见 CONTROL_ATTITUDE_QUATERNION)
 *  符号约定与 INS.c 中的赋值保持一致:
 *      INS.Roll         = +绕机体X轴(标准横滚角)      -> Feedback_P_Roll
 *      INS.Pitch        = -绕机体Y轴(标准俯仰角取反)  -> Feedback_P_Pitch
 *      INS.YawTotalAngle= 累计总偏航角(不会在 ±180° 跳变) -> Feedback_P_Yaw
 *============================================================================*/

#if (CONTROL_ATTITUDE_QUATERNION == 1u)
/**
 * @brief  由目标欧拉角(Control.Target_Yaw/Pitch/Roll, 单位deg)生成目标姿态四元数
 * @param  q 输出四元数(w,x,y,z), 与 INS.q 同一约定(机体系->导航系)
 * @note   按标准 ZYX(偏航-俯仰-横滚)次序, 其中俯仰取 -Target_Pitch
 */
static void Control_GetTargetQuaternion(float *q)
{
    float cy = cosf(Control.Target_Yaw  * 0.5f * Angle2Rad);
    float sy = sinf(Control.Target_Yaw  * 0.5f * Angle2Rad);
    float cp = cosf(-Control.Target_Pitch * 0.5f * Angle2Rad);   /* cos(-x)=cos(x)  */
    float sp = sinf(-Control.Target_Pitch * 0.5f * Angle2Rad);   /* sin(-x)=-sin(x) */
    float cr = cosf(Control.Target_Roll * 0.5f * Angle2Rad);
    float sr = sinf(Control.Target_Roll * 0.5f * Angle2Rad);

    q[0] = cy * cp * cr + sy * sp * sr;     /* w */
    q[1] = cy * cp * sr - sy * sp * cr;     /* x: 横滚轴 */
    q[2] = cy * sp * cr + sy * cp * sr;     /* y: 俯仰轴 */
    q[3] = sy * cp * cr - cy * sp * sr;     /* z: 偏航轴 */
}

/**
 * @brief  计算姿态误差, 结果写入 Control.Err_Angle[3](单位 deg)
 * @note   q_err = q_cur^-1 ⊗ q_target, 其旋转矢量就是"当前姿态 -> 目标姿态"
 *         所需的机体轴转角(导航系误差已转到机体轴), 无 ±90° 奇异。
 */
static void Control_CalcAttitudeError(void)
{
    const float *qc = Control.Feedback_Q;
    float q_t[4];
    float qe[4];
    float s, k;

    Control_GetTargetQuaternion(q_t);
    memcpy(Control.Target_Q, q_t, sizeof(q_t));

    /* qe = q_cur^-1 (共轭) ⊗ q_target */
    qe[0] =  qc[0] * q_t[0] + qc[1] * q_t[1] + qc[2] * q_t[2] + qc[3] * q_t[3];
    qe[1] =  qc[0] * q_t[1] - qc[1] * q_t[0] - qc[2] * q_t[3] + qc[3] * q_t[2];
    qe[2] =  qc[0] * q_t[2] + qc[1] * q_t[3] - qc[2] * q_t[0] - qc[3] * q_t[1];
    qe[3] =  qc[0] * q_t[3] - qc[1] * q_t[2] + qc[2] * q_t[1] - qc[3] * q_t[0];

    /* 取最短旋转(qe[0]>=0 ⇒ 转角 ∈ [0,180°]) */
    if (qe[0] < 0.0f)
    {
        qe[0] = -qe[0];
        qe[1] = -qe[1];
        qe[2] = -qe[2];
        qe[3] = -qe[3];
    }

    /* 旋转矢量 = 转轴 * 转角(deg) */
    s = sqrtf(qe[1] * qe[1] + qe[2] * qe[2] + qe[3] * qe[3]);
    k = (s > 1e-6f) ? (2.0f * atan2f(s, qe[0]) * Rad2Angle / s) : 0.0f;

    /* 机体轴 -> 控制通道(俯仰通道符号与 INS.Pitch 一致) */
    Control.Err_Angle[0] =  qe[1] * k;      /* Roll  : +绕机体X */
    Control.Err_Angle[1] = -qe[2] * k;      /* Pitch : -绕机体Y */
    Control.Err_Angle[2] =  qe[3] * k;      /* Yaw   : +绕机体Z */
}
#endif  /* CONTROL_ATTITUDE_QUATERNION */

/**
 * @brief  Acro 模式摇杆曲线: 死区 + Expo(只用在中段手感, 不影响满杆量)
 * @param  Stick 摇杆量(-1~+1)
 * @return 处理后的摇杆量(-1~+1)
 */
static float Control_AcroCurve(float Stick)
{
    float y;

    if (Stick >  1.0f) Stick =  1.0f;
    if (Stick < -1.0f) Stick = -1.0f;

    /* 死区内的杆量视为 0(避免摇杆回中不精确时一直微转) */
    if (fabsf(Stick) <= CONTROL_ACRO_DEADBAND) return 0.0f;

    /* 死区外重新映射到 0~1, 保证满杆仍为 1 */
    if (Stick > 0.0f) y = (Stick - CONTROL_ACRO_DEADBAND) / (1.0f - CONTROL_ACRO_DEADBAND);
    else              y = (Stick + CONTROL_ACRO_DEADBAND) / (1.0f - CONTROL_ACRO_DEADBAND);

    /* Expo: y = (1-e)*y + e*y^3  (y^3 保号) */
    return (1.0f - CONTROL_ACRO_EXPO) * y + CONTROL_ACRO_EXPO * y * y * y;
}

/**
 * @brief  油门杆(-1~1) -> 推力域指令(0~100), 曲线见 CONTROL_THROTTLE_CURVE
 * @param  RC_Thrust 油门杆量(-1~+1)
 */
static float Control_ThrottleCurve(float RC_Thrust)
{
    float s = (RC_Thrust + 1.0f) * 0.5f;      /* 摇杆 -> 0~1 */

    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;

#if (CONTROL_THROTTLE_CURVE == 1u)
    return 100.0f * s * s;                    /* 推力域 T=100s² -> 混控里 duty=10√T=100s */
#else
    return 100.0f * s;                        /* 原行为: 推力与杆量线性(中位=70% duty) */
#endif
}

void Control_Task(void *argument)
{
    osDelay(100);
    Control.Target_Yaw = INS.YawTotalAngle;
    Control.V_Comp = 1.0f;
    
    /*===| 高度环: 高度 P -> 速度 P -> 推力(带 Hover_Thrust 前馈) |===*/
    PID_Init(&Control.PID_P_Height, CONTROL_HEIGHT_KP, 0, CONTROL_HEIGHT_KD, 0, 0, CONTROL_HEIGHT_OUT_MAX);
    PID_Init(&Control.PID_S_Height, CONTROL_HEIGHT_SPEED_KP, CONTROL_HEIGHT_SPEED_KI, 0, 0, CONTROL_HEIGHT_SPEED_I_MAX, CONTROL_HEIGHT_SPEED_OUT_MAX);

    /*===| 姿态外环(角度 -> 期望角速度): 只用 P, D/I 全放到角速度内环 |===*/
    PID_Init(&Control.PID_P_Roll,  CONTROL_ANGLE_KP_RP,  0, 0, 0, 0, CONTROL_RATE_SP_MAX);
    PID_Init(&Control.PID_P_Pitch, CONTROL_ANGLE_KP_RP,  0, 0, 0, 0, CONTROL_RATE_SP_MAX);
    PID_Init(&Control.PID_P_Yaw,   CONTROL_ANGLE_KP_YAW, 0, 0, 0, 0, CONTROL_RATE_SP_MAX);

    /*===| 角速度内环(角速度 -> 推力差动): P + I + D(测量微分) + 速率前馈 |===*/
    PID_Init(&Control.PID_S_Roll,  CONTROL_RATE_KP,     CONTROL_RATE_KI,     CONTROL_RATE_KD,     CONTROL_RATE_KF,     CONTROL_RATE_I_MAX, CONTROL_RATE_OUT_MAX);
    PID_Init(&Control.PID_S_Pitch, CONTROL_RATE_KP,     CONTROL_RATE_KI,     CONTROL_RATE_KD,     CONTROL_RATE_KF,     CONTROL_RATE_I_MAX, CONTROL_RATE_OUT_MAX);
    PID_Init(&Control.PID_S_Yaw,   CONTROL_RATE_KP_YAW, CONTROL_RATE_KI_YAW, CONTROL_RATE_KD_YAW, CONTROL_RATE_KF_YAW, CONTROL_YAW_I_MAX,   CONTROL_RATE_OUT_MAX);
    PID_Set_D_LPF(&Control.PID_S_Roll,  CONTROL_RATE_D_LPF_HZ);
    PID_Set_D_LPF(&Control.PID_S_Pitch, CONTROL_RATE_D_LPF_HZ);
    PID_Set_D_LPF(&Control.PID_S_Yaw,   CONTROL_RATE_D_LPF_HZ);

    /*===| 定点模式(速度 -> 姿态角) |===*/
    PID_Init(&Control.PID_S_X, CONTROL_VEL_KP, CONTROL_VEL_KI, 0, 0, CONTROL_VEL_I_MAX, CONTROL_VEL_OUT_MAX);
    PID_Init(&Control.PID_S_Y, CONTROL_VEL_KP, CONTROL_VEL_KI, 0, 0, CONTROL_VEL_I_MAX, CONTROL_VEL_OUT_MAX);
    

    
    /*===| 电调输出初始化: DShot600 (TIM4 CH1~CH4 + DMA) |===*/
    DShot_Init();
    
    DWT_GetDeltaT(&Control_DWT_Couter);
    for(;;)
    {		
        Control_Dt = DWT_GetDeltaT(&Control_DWT_Couter);
        
        /* 解锁条件: 遥控已连接 + SA 开关有效 + 陀螺零漂本次上电已实测成功。
         * 最后一条是硬条件: 不允许用固化的零漂值起飞(没标定好就锁死, 电机不起转)。
         * 未标定时保持静止约 0.5s 即可自动完成, 成功后 
         * Gyro_Cal_OK 与 Control.Enable 才会置 1。
         * (IMU_CAL_GYRO_CAL_EN=0 时 Is_Valid() 恒为 1, 即不做限制) */
        Control.Gyro_Cal_OK = IMU_Cal_Gyro_Bias_Is_Valid();
        Control.Enable = (Remote.If_Remote_Connect && (Remote.SA != -1) && Control.Gyro_Cal_OK) ? 1u : 0u;
        
        if(Remote.SB == -1)                   //自稳(角度模式): 摇杆给目标姿态角, 姿态外环维持水平
            Control.Control_Mode = Control_Mode_Level;
        else if(Remote.SB == 0)               //定高
            Control.Control_Mode = Control_Mode_Level_Height;
        else if(Remote.SB == 1)               //定点
            Control.Control_Mode = Control_Mode_Level_Point;

        //////////////////////////////  控制量输入  //////////////////////////////
        float RC_Thrust,RC_Yaw,RC_Pitch,RC_Roll;
        if(Remote.SC < 0) //日本手
        {
            RC_Thrust = Remote.Right_Y;
            RC_Yaw    = Remote.Right_X;
            RC_Pitch  = Remote.Left_Y;
            RC_Roll   = Remote.Left_X;
        }
        else //美国手
        {
            RC_Thrust = Remote.Left_Y;
            RC_Yaw    = Remote.Left_X;
            RC_Pitch  = Remote.Right_Y;
            RC_Roll   = Remote.Right_X;
        }
        
        if(Control.Control_Mode == Control_Mode_Level) //自稳(角度模式): 摇杆给目标姿态角
        {   
            Control.Target_Height = Control_ThrottleCurve(RC_Thrust);
            Control.Target_Pitch = -RC_Pitch * 60.0f;
            Control.Target_Roll = RC_Roll * 60.0f;
            Control.Target_Yaw -= Control_Dt * RC_Yaw * 300.0f;
        }
        else if(Control.Control_Mode == Control_Mode_Acro) //Acro: 摇杆直接给期望角速度
        {
            /* 注: 默认不从 SB 开关进入(当前 SB=-1 是自稳); 想把最低位切回 Acro,
             * 只改上面的模式选择即可 */
            Control.Target_Height = Control_ThrottleCurve(RC_Thrust);      /* 总距直接跟油门杆 */

            /* 符号与角度模式的“目标姿态->期望角速度”一致(实机上先把方向验对):
             *   通道0(Roll)  = +绕机体X   -> +杆=正向滚
             *   通道1(Pitch) = -绕机体Y   -> +杆=负向俯(与 RC_Pitch 反号, 同角度模式) */
            Control.Rate_Stick[0] =  CONTROL_ACRO_RATE_RP  * Control_AcroCurve(RC_Roll);
            Control.Rate_Stick[1] = -CONTROL_ACRO_RATE_RP  * Control_AcroCurve(RC_Pitch);
            Control.Rate_Stick[2] = -CONTROL_ACRO_RATE_YAW * Control_AcroCurve(RC_Yaw);

            /* Acro 下没有航向保持, 把航向目标追到当前值 -> 切回角度模式时不会突然转头 */
            Control.Target_Yaw = INS.YawTotalAngle;
        }
        else if(Control.Control_Mode == Control_Mode_Level_Height) //定高模式
        {
            Control.Target_Height += Control_Dt * Remote.Right_Y * 1000.0f;
            Control.Target_Pitch = -RC_Pitch * 60.0f;
            Control.Target_Roll = RC_Roll * 60.0f;
            Control.Target_Yaw -= Control_Dt * RC_Yaw * 90.0f;
        }
        else if(Control.Control_Mode == Control_Mode_Level_Point) //定点模式
        {
            Control.Target_Height += Control_Dt * Remote.Right_Y * 1000.0f;
            Control.Target_V_X = Remote.Left_X * 200.0f;
            Control.Target_V_Y = Remote.Left_Y * 200.0f;
            Control.Target_Yaw -= Control_Dt * Remote.Right_X * 90.0f;
        }
            
        if(Remote.SA <= -0.5)
        {
            Control.Target_Yaw = INS.YawTotalAngle;  
            /* Acro 下总距必须完全跟手, 不被高度传感器接管 */
            if(Control.Control_Mode != Control_Mode_Acro)
                Control.Target_Height = Sensor.Height_mm + 100;
        }
        
        //////////////////////////////  反馈量输入  //////////////////////////////
        Control.Feedback_P_Height = Sensor.Height_mm;
        /* 两种姿态反馈都取上: 四元数路径用 Feedback_Q, 欧拉角路径用下面三个
         * (都取上也可方便 Vofa 直接看姿态角/总角度) */
        Control.Feedback_P_Roll  = INS.Roll;
        Control.Feedback_P_Pitch = INS.Pitch;
        Control.Feedback_P_Yaw   = INS.YawTotalAngle;      /* totalangle */
        memcpy(Control.Feedback_Q, INS.q, sizeof(Control.Feedback_Q));

        Control.Feedback_S_Height = Sensor.Height_Speed_mm;
        Control.Feedback_S_Pitch = INS.Pitch_Speed;
        Control.Feedback_S_Roll = INS.Roll_Speed;
        Control.Feedback_S_Yaw = INS.Yaw_Speed;

        Control.Feedback_V_X = Sensor.Vel_X_mm;
        Control.Feedback_V_Y = Sensor.Vel_Y_mm;
        
        //////////////////////////////  一键滚转  //////////////////////////////
    #if CONTROL_AUTO_ROLL_ENABLE
        /* SD 按下(通道8 置高) -> 自动滚转一圈
         * ⚠ SBUS 端点是 ±1.03/±1.01, 不能用 Remote.SD == 1.0f 判断(永远不成立) */
        {
            /* 初值给1: 要求 SD 必须先回到低位, 避免上电时开关就在高位 -> 一解锁就自己翻 */
            static uint8_t SD_Last = 1;
            uint8_t SD_Now = (Remote.SD > 0.5f) ? 1u : 0u;

            if (Control.Auto_Roll_State == 1)
            {
                /* 用真实角速度积分"实际转过的角度"(不依赖增益/是否跟得上) */
                Control.Auto_Roll_Time  += Control_Dt;
                Control.Auto_Roll_Angle += fabsf(Control.Feedback_S_Roll) * Control_Dt;

                if ((Control.Auto_Roll_Angle >= CONTROL_AUTO_ROLL_ANGLE) ||    /* 转够一圈   */
                    (Control.Auto_Roll_Time  >  CONTROL_AUTO_ROLL_TIMEOUT) ||  /* 超时保护   */
                    (!Control.Enable))                                         /* 中途上锁   */
                {
                    Control.Auto_Roll_State = 0;
                }
            }
            else if (SD_Now && (SD_Last == 0) && Control.Enable && (INS.zn[2] > CONTROL_AUTO_ROLL_START_COS))
            {
                /* 上升沿起转: 需已解锁且接近水平, 防误触/倒扣时起转 */
                Control.Auto_Roll_State = 1;
                Control.Auto_Roll_Angle = 0.0f;
                Control.Auto_Roll_Time  = 0.0f;
            }
            SD_Last = SD_Now;

            if (Control.Auto_Roll_State == 1)
            {
                /* 期望角速度: 剩余角度不足 TAPER 时按比例降速(保底 MIN_SCALE), 避免急停冲过头 */
                float Remaining  = CONTROL_AUTO_ROLL_ANGLE - Control.Auto_Roll_Angle;
                float Rate_Scale = Remaining / CONTROL_AUTO_ROLL_TAPER;
                if (Rate_Scale > 1.0f)  Rate_Scale = 1.0f;
                if (Rate_Scale < CONTROL_AUTO_ROLL_MIN_SCALE) Rate_Scale = CONTROL_AUTO_ROLL_MIN_SCALE;
                Control.Auto_Roll_Rate = CONTROL_AUTO_ROLL_DIR * CONTROL_AUTO_ROLL_RATE * Rate_Scale;
            }
            else
            {
                Control.Auto_Roll_Rate = 0.0f;
            }
        }
    #endif
        
        //////////////////////////////  PID计算  //////////////////////////////
        /*=== 位置环 ===*/
        if(Control.Control_Mode == Control_Mode_Acro)  //Acro: 高度环不参与, 总距直接跟油门杆
        {
            PID_Clear_Output(&Control.PID_P_Height);
            PID_Clear_Output(&Control.PID_S_Height);
        }
        else if(Control.Control_Mode == Control_Mode_Level || Control.Control_Mode == Control_Mode_Level_Height) //手动或定高
        {
            PID_Position_Calculate(&Control.PID_P_Height, Control.Target_Height, Control.Feedback_P_Height);
        }
        else if(Control.Control_Mode == Control_Mode_Level_Point)               //定点模式
        {
            PID_Position_Calculate(&Control.PID_S_X, Control.Target_V_X, Control.Feedback_V_X);
            PID_Position_Calculate(&Control.PID_S_Y, Control.Target_V_Y, Control.Feedback_V_Y);

            PID_Position_Calculate(&Control.PID_P_Height, Control.Target_Height, Control.Feedback_P_Height);
            /* 位置环输出 -> 姿态目标(与原来一致), 交给下面的姿态外环 */
            Control.Target_Pitch = -Control.PID_S_Y.Output;
            Control.Target_Roll  =  Control.PID_S_X.Output;
        }

        /*=== 姿态外环: 姿态误差 -> 期望角速度 ===*/
    #if (CONTROL_ATTITUDE_QUATERNION == 1u)
        /* 四元数误差: Target=误差, Feedback=0 (等价于 Target-Feedback) */
        Control_CalcAttitudeError();
        PID_Position_Calculate(&Control.PID_P_Roll,  Control.Err_Angle[0], 0.0f);
        PID_Position_Calculate(&Control.PID_P_Pitch, Control.Err_Angle[1], 0.0f);
        PID_Position_Calculate(&Control.PID_P_Yaw,   Control.Err_Angle[2], 0.0f);
    #else
        /* 欧拉角: 普通 Target - Feedback(偏航用累计总角度) */
        PID_Position_Calculate(&Control.PID_P_Roll,  Control.Target_Roll,  Control.Feedback_P_Roll);
        PID_Position_Calculate(&Control.PID_P_Pitch, Control.Target_Pitch, Control.Feedback_P_Pitch);
        PID_Position_Calculate(&Control.PID_P_Yaw,   Control.Target_Yaw,   Control.Feedback_P_Yaw);
    #endif

        /*=== 摇杆前馈: 目标角度的变化率 x FF 直接叠加到期望角速度 ===
         * 打杆瞬间就给出期望转速, 不必等角度误差慢慢积累 -> 跟手且线性,
         * 外环 P 可以调小(越小越不易振荡)。
         * 目标被“直接赋值”造成跳变时(如 SA 回中重置航向)该拍不做前馈 */
        if (Control_Dt > 1e-5f)
        {
            float dT[3];
            dT[0] = Control.Target_Roll  - Control.Target_Last[0];
            dT[1] = Control.Target_Pitch - Control.Target_Last[1];
            dT[2] = Control.Target_Yaw   - Control.Target_Last[2];

            for (uint8_t i = 0; i < 3; i++)
            {
                if ((dT[i] > CONTROL_ANGLE_FF_JUMP) || (dT[i] < -CONTROL_ANGLE_FF_JUMP))
                    Control.Rate_FF[i] = 0.0f;                           /* 跳变: 不前馈 */
                else
                    Control.Rate_FF[i] = dT[i] / Control_Dt * CONTROL_ANGLE_FF;
            }
        }
        else
        {
            Control.Rate_FF[0] = 0.0f;
            Control.Rate_FF[1] = 0.0f;
            Control.Rate_FF[2] = 0.0f;
        }
        Control.Target_Last[0] = Control.Target_Roll;
        Control.Target_Last[1] = Control.Target_Pitch;
        Control.Target_Last[2] = Control.Target_Yaw;

        /*=== 期望角速度 ===
         * Acro : 摇杆直接就是期望角速度(姿态外环不参与)
         * 其它 : 外环 P + 摇杆前馈
         * 注: 外环与摇杆前馈在 Acro 下仍然在算(保持内部状态新鲜), 只是不被采用,
         *     这样切换模式时不会因为滤波器/积分器突变而抽一下 */
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
        Limit_float(&Control.Rate_SP[0],  CONTROL_RATE_SP_MAX, -CONTROL_RATE_SP_MAX);
        Limit_float(&Control.Rate_SP[1],  CONTROL_RATE_SP_MAX, -CONTROL_RATE_SP_MAX);
        Limit_float(&Control.Rate_SP[2],  CONTROL_RATE_SP_MAX, -CONTROL_RATE_SP_MAX);

    #if CONTROL_AUTO_ROLL_ENABLE
        /* 滚转中: 直接用期望角速度覆盖“外环+前馈”的结果, 交给角速度内环执行。
         * 外环仍在跑, 退出时不会因为微分项突变而抽一下 */
        if (Control.Auto_Roll_State == 1)
            Control.Rate_SP[0] = Control.Auto_Roll_Rate;
    #endif

        /*=== 角速度内环: P + I + D(测速微分) + 速率前馈 ===*/
        PID_Position_Calculate(&Control.PID_S_Height, Control.PID_P_Height.Output, Control.Feedback_S_Height);
        PID_Position_Calculate(&Control.PID_S_Roll,   Control.Rate_SP[0], Control.Feedback_S_Roll);
        PID_Position_Calculate(&Control.PID_S_Pitch,  Control.Rate_SP[1], Control.Feedback_S_Pitch);
        PID_Position_Calculate(&Control.PID_S_Yaw,    Control.Rate_SP[2], Control.Feedback_S_Yaw);
        
        if(Control.Control_Mode == Control_Mode_Level || Control.Control_Mode == Control_Mode_Acro) //手动: 总距跟油门杆
            Control.T_Height = Control.Target_Height;
        else if(Control.Control_Mode == Control_Mode_Level_Height || Control.Control_Mode == Control_Mode_Level_Point)               //定高或定点
            Control.T_Height = Hover_Thrust + Control.PID_S_Height.Output;
        Limit_float(&Control.T_Height, 100.0f, 0.0f);
        
        Control.T_Pitch = Control.PID_S_Pitch.Output;
        Control.T_Roll = Control.PID_S_Roll.Output;
        Control.T_Yaw = Control.PID_S_Yaw.Output;

        //////////////////////////////  倾斜油门补偿  //////////////////////////////
    #if CONTROL_TILT_COMPENSATION
        /* cos(倾角) = cos(Roll)*cos(Pitch) */
        float cos_t = cosf(INS.Roll * Angle2Rad) * cosf(INS.Pitch * Angle2Rad);
        if (cos_t < CONTROL_MIN_COS_TILT) cos_t = CONTROL_MIN_COS_TILT;
        Control.T_Height = Control.T_Height / cos_t;   // 倾斜自动补油
  
        Limit_float(&Control.T_Height, 100.0f, 0.0f);      // 补偿后再夹一次
    #endif

        //////////////////////////////  混控  //////////////////////////////
        
        float diff[4];
        float dmax, dmin, range, s = 1.0f;
        float base, base_min, base_max;

        /* 机架: 十字(+)型 —— 四个机臂分别朝 左/后/右/前
         *       电机1=左臂(CW)  电机2=后臂(CCW)  电机3=右臂(CW)  电机4=前臂(CCW)
         *       CW/CCW = 俯视桨的旋转方向
         * 姿态差动(不含总距)。信号约定与旧 X 型混控完全一致,
         * 因此 PID 参数、遥控方向、姿态环符号都不需要改:
         *   T_Pitch > 0 -> 前臂加速、后臂减速 -> 抬头
         *   T_Roll  > 0 -> 左臂加速、右臂减速 -> 右滚
         *   T_Yaw   > 0 -> 顺时针(CW)桨加速 -> 机身反扭逆时针 -> 左偏航
         * 规律: 俯仰 前+ 后-, 横滚 左+ 右-, 偏航 CW+ CCW- */
        diff[0] =  Control.T_Roll  + Control.T_Yaw;      /* 1号 左臂 CW  */
        diff[1] = -Control.T_Pitch - Control.T_Yaw;      /* 2号 后臂 CCW */
        diff[2] = -Control.T_Roll  + Control.T_Yaw;      /* 3号 右臂 CW  */
        diff[3] =  Control.T_Pitch - Control.T_Yaw;      /* 4号 前臂 CCW */

        dmax = diff[0]; dmin = diff[0];
        for (uint8_t i = 1; i < 4; i++)
        {
            if (diff[i] > dmax) dmax = diff[i];
            if (diff[i] < dmin) dmin = diff[i];
        }

        /* 差动幅度本身超过全量程(极端目标) -> 等比压缩差动 */
        range = dmax - dmin;
        if (range > 100.0f) s = 100.0f / range;

        /* 姿态优先: 差动完整保留, 平移总距使四路都落进 [0,100] */
        base_min = -s * dmin;          if (base_min < 0.0f)    base_min = 0.0f;
        base_max = 100.0f - s * dmax;  if (base_max > 100.0f)  base_max = 100.0f;
        if (base_max < base_min) base_max = base_min;      /* 数值保护 */
        base = Control.T_Height;
        if (base > base_max) base = base_max;
        if (base < base_min) base = base_min;

        for (uint8_t i = 0; i < 4; i++)
        {
            Control.Thrust[i] = base + s * diff[i];
            /* 解锁后给电机怠速下限, 避免空中停转 */
            if (Control.Enable && (Control.Thrust[i] < CONTROL_MOTOR_IDLE))
                Control.Thrust[i] = CONTROL_MOTOR_IDLE;
            Limit_float(&Control.Thrust[i], 100.0f, 0.0f);
        }
        
        
        //////////////////////////////  输出  //////////////////////////////
        /* 电压补偿: 推力∝V², 掉压时把占空比按 V_REF/Vbat 放大回来 */
        Control.Vbat = Sensor.BatV;
    #if CONTROL_V_COMP_ENABLE
        if (Control.Vbat > CONTROL_VBAT_MIN)
        {
            Control.V_Comp = CONTROL_V_REF / Control.Vbat;
            Limit_float(&Control.V_Comp, CONTROL_V_COMP_MAX, CONTROL_V_COMP_MIN);
        }
        else
    #endif
        {
            Control.V_Comp = 1.0f;      /* 电压无效 -> 不补偿 */
        }
        
        if(Control.Enable)
        {
            for(uint8_t i = 0; i < 4; i++)
                Control_Set_Thrust(i, Control.Thrust[i]);   /* 推力->占空比(曲线反解+电压补偿) */
        }
        else
        {
            for(uint8_t i = 0; i < 4; i++)
            {
                Control.Thrust[i] = 0.0f;
                Control_Set_Duty(i, 0);
            }
            
            PID_Clear_Output(&Control.PID_S_Height);
            PID_Clear_Output(&Control.PID_S_Pitch);
            PID_Clear_Output(&Control.PID_S_Roll);
            PID_Clear_Output(&Control.PID_S_Yaw);
        }
        
        /* 4 路一起发一帧 DShot(约 1ms 一次, 电调靠连续收帧判断信号正常) */
        DShot_Send();
        
        osDelay(1);
    }
}

/* 推力指令(0~100, 归一化推力域) -> 占空比(0~100)
 *   1) 反解推力曲线: 推力∝占空比²  =>  duty = 100*sqrt(thrust/100)
 *   2) 电压补偿:     推力∝电压²    =>  duty *= V_REF/Vbat
 *   3) 限幅
 * 说明: 输出不再是占空比 PWM, 而是 DShot600 油门值(见 DShot.c) */
void Control_Set_Thrust(uint8_t Fan_ID, float Thrust_100)
{
    float Thrust_Temp = Thrust_100;
    float Duty_Temp;
    
    Limit_float(&Thrust_Temp, 100.0f, 0.0f);
    
    /* 推力 ∝ 占空比² 的反解(空心杯+桨, 电压已由 V_Comp 单独补偿) */
    Duty_Temp = 100.0f * sqrtf(Thrust_Temp / 100.0f);
    Duty_Temp *= Control.V_Comp;      /* 电压补偿 */
    
    Control_Set_Duty(Fan_ID, Duty_Temp);
}

/* 占空比指令(0~100) -> DShot 油门值(48~2047)
 *   只在帧缓冲里更新, 真正的发送由 DShot_Send() 统一触发(4 路同一帧) */
void Control_Set_Duty(uint8_t Fan_ID, float Duty_100)
{
    float Duty_Temp = Duty_100;
    Limit_float(&Duty_Temp, Duty_Max, 0);
    if (Fan_ID < 4) Control.Duty[Fan_ID] = Duty_Temp;   /* 便于遥测/日志 */
    DShot_Set_Duty(Fan_ID, Duty_Temp);
        
}