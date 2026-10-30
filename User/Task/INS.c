#include "INS.h"

#include "Parameter.h"
#include "QuaternionEKF.h"
#include "BMI088.h"
#include "IMU_Cal.h"
#include "Buzzer.h"
#include "Filter.h"

INS_t INS;
IMU_Param_t IMU_Param;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
/* 调试/验证用: 最近一个陀螺样本的真实间隔(us)。正常应在
 * 1000000/BMI088_GYRO_ODR_HZ ≈ 500us 附近小幅波动; 恒定不变说明 INT 没生效,
 * 一直在走 20ms 兜底轮询。 */
volatile uint32_t INS_Sample_Dt_Us = 0;
#if (INS_SAMPLE_TS_EN == 1u)
static uint32_t INS_Gyro_Seq_Last = 0;   /* 上次处理过的陀螺样本序号(判“有没有新样本”) */
#endif

/*==============================================================================
 *  陀螺滤波链(抑制电机/桨振动): 原始 -> 陷波1 -> 陷波2 -> 二阶低通 ->(可选)一阶低通
 *
 *  为什么上桨后“飘/抖”: 桨/电机振动会被内环 D 项和混控放大,
 *  表现为陀螺读数大幅波动、姿态发抖发飘。
 *  陷波只削那几个窄带, 对环路延迟的影响远小于“一味加大低通”。
 *
 *  ★ 实测(2026-10-19, 2kHz 采原始陀螺做 FFT): 两个共振峰在 **271Hz 和 657Hz**,
 *    半带宽约 ±20Hz / ±30Hz -> 用两级**窄**陷波分别对准。
 *    不用一个宽陷波罩住, 是因为 657±250 这种坑会把 250~500Hz 控制带的相位一起削掉。
 *  ★ 换了桨/机架要重新测: 把 NOTCH/LP 先关掉、2kHz 录原始陀螺做 FFT, 取峰重填。
 *    Nyquist 只有 1kHz, 峰高于 900Hz 的只能靠芯片 LPF(CTRL5)去压。
 *  Q 的取法: BW = F0/Q -> Q = F0/BW
 *     271Hz ± 20Hz(BW 40) -> Q = 6.8   (坑 251~291Hz)
 *     657Hz ± 30Hz(BW 60) -> Q = 11    (坑 627~687Hz)
 *  ⚠ 窄陷波只在测频谱那个转速准; 若峰随油门成比例移动, 就得换动态陷波/RPM 滤波。
 *============================================================================*/
/* 全部滤波参数见 Parameter.h 第 2 节「IMU 滤波 / 姿态解算」 */

/* 角速度反馈再串一级一阶低通(兜底): 0=关
 * 注: 0.01(=16Hz) 太滞后, 会让内环发软发飘; 有陷波后这里保持 0 即可 */

/* ── EKF 观测(加速度计)低通时间常数(s): 0 = 完全不过滤 ──
 * 为什么不能用 0: 桨/电机振动会把加速度矢量在 ±1g 以上调制, 直接送进 EKF 后,
 *   观测残差 r = |实测重力方向 - 姿态估计出的重力方向| 变大; 一旦超过
 *   QuaternionEKF.c 里的卡方检验阈值(1e-8, 相当于方向误差约 5.7°), 该帧就会走
 *   “残差未通过卡方检验 -> 仅预测” 分支 ── 整帧跳过加速度计修正,
 *   姿态退化成纯陀螺积分。表现就是: 电机一抖欧拉角大幅漂移, 停机后又自己回正。
 * 取 0.01s ≈ 16Hz: 保留重力(直流)方向, 压掉几十~几百 Hz 的振动,
 *   残差回到阈值内, 振动期间加速度计修正照常生效。
 * ★ 4kHz 下改成 0.005 会让 200Hz 以上的振动多漏一倍进 EKF(实测上桨后
 *   pitch/roll 偏十几度就是这条链被踩中), 所以这里保持 0.01。
 *   需更快跟随倾斜时可降到 0.005, 但同时必须把 QEKF_ADAPT_GAIN_MIN 抬上去。
 * 注意: 它与 INS.AccelLPF(只用于算运动加速度)无关, 别搞混。 */

static float INS_Speed_LP[3] = {0.0f, 0.0f, 0.0f};   /* 3路一阶低通状态(芯片原始轴向) [0]=GyroX [1]=GyroY [2]=GyroZ */
#if (INS_GYRO_NOTCH_EN == 1u)
static Biquad_t INS_Gyro_Notch[3];
#endif
#if (INS_GYRO_NOTCH2_EN == 1u)
static Biquad_t INS_Gyro_Notch2[3];
#endif
#if (INS_GYRO_LP_EN == 1u)
static Biquad_t INS_Gyro_LP[3];
#endif
#if (INS_GYRO_FILTER_EKF == 1u)
static float INS_Gyro_F[3];                          /* 滤波后角速度(rad/s, 原始轴向), 给 EKF */
#endif


void INS_Init(void)
{
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    /* 末位参数 = EKF 内部对加速度计的一阶低通时间常数(s), 0 = 不过滤
     * 详见文件顶部 INS_EKF_ACCEL_LPF_TAU 的说明 */
    IMU_QuaternionEKF_Init(QEKF_PROCESS_NOISE1, QEKF_PROCESS_NOISE2, QEKF_MEASURE_NOISE,
                           QEKF_LAMBDA, INS_EKF_ACCEL_LPF_TAU);

    INS.AccelLPF = 0.0085;

    INS.Yaw_Speed  = 0;
    INS.Pitch_Speed = 0;
    INS.Roll_Speed  = 0;

    /* 陀螺滤波初始化(陷波/低通系数按标称采样率算一次) */
    for (uint8_t i = 0; i < 3; i++)
    {
        INS_Speed_LP[i] = 0.0f;
#if (INS_GYRO_LP_EN == 1u)
        Biquad_LowPass_Init(&INS_Gyro_LP[i], INS_GYRO_LP_FC, INS_FILTER_FS_HZ, 0.7071f);
#endif
#if (INS_GYRO_NOTCH_EN == 1u)
        Biquad_Notch_Init(&INS_Gyro_Notch[i], INS_GYRO_NOTCH_F0, INS_FILTER_FS_HZ, INS_GYRO_NOTCH_Q);
#endif
#if (INS_GYRO_NOTCH2_EN == 1u)
        Biquad_Notch_Init(&INS_Gyro_Notch2[i], INS_GYRO_NOTCH2_F0, INS_FILTER_FS_HZ, INS_GYRO_NOTCH2_Q);
#endif
    }
}



/*============================================================================
 *  INS 初始化(只跑一次, 由 Flight_Task 调用)
 *===========================================================================*/
void INS_Init_All(void)
{
    INS_Init();
    BMI088_Init();

    // 上电只认 Flash 里保存的标定参数;
    // Flash 里没有(或记录损坏)时才会跑一次交互式 IMU_Cal(), 结束后自动写回 Flash。
    // 需要强制重新标定: 先 IMU_Cal_Reset_Config() + IMU_Cal_Save_Config() 抹掉记录再重启。
    if (!IMU_Cal_Load_Config())
    {
        IMU_Cal();
    }

    // 上电陀螺零漂校准: 改为后台运行(见 INS_Update 里的 IMU_Cal_Gyro_Bias_Update),
    // 不再阻塞等待静止, 避免上电时被搬动/振动就把初始化卡住。

    /* 预置 DWT 计时起点: 否则第一拍 dt 会是"上电到现在"的一个大值 */
    DWT_GetDeltaT(&INS_DWT_Count);
}

/*============================================================================
 *  一拍姿态解算(由 Flight_Task 在 TIM6 节拍里调用, 内部不再自带延时)
 *
 *  ★ 精度关键: 积分只在“有新陀螺样本”时推进一次, dt 用 EXTI 记下的硬件
 *    时间戳算真实样本间隔(见 BMI088.Gyro_Dt_Cyc / INS_SAMPLE_TS_EN)。
 *    传感器 ODR(2000Hz) 与 TIM6 节拍(2000Hz) 不是同一个时钟源, 若死用固定
 *    dt, 会出现某个样本被用 1 次或 3 次 -> 积分总时长偶尔多/少半拍。
 *===========================================================================*/
void INS_Update(void)
{
    const float gravity[3] = {0, 0, 9.8f};

    /* 先维护 DWT 循环计时(INS_DWT_Count 仍可用于观测循环抖动) */
    dt = DWT_GetDeltaT(&INS_DWT_Count);

    // ins update
    IMU_Cal_Get_Calibrated_Data(INS.Accel, INS.Gyro);   // 标定完成后改回此行即可

#if (INS_SAMPLE_TS_EN == 1u)
    /* ★ 判新样本必须放在读数据之后: BMI088_Read_Data() 内部才递增 Gyro_Seq。
     * 没有新样本时不重复积分、也不重复滤波 —— 把同一份数据当两个样本再跑
     * 一遍滤波, 等效于把截止频率压低一半, 只会多出相位滞后。 */
    if (BMI088.Gyro_Seq == INS_Gyro_Seq_Last)
    {
        return;                       /* 输出保持上一拍的值, 控制环照常用 */
    }
    INS_Gyro_Seq_Last = BMI088.Gyro_Seq;

    dt = DWT_Cyc_To_S(BMI088.Gyro_Dt_Cyc);            /* 老样本之间的真实间隔 */
    /* 首个样本(没有间隔)/兜底轮询/时间戳异常 -> 退回理论采样间隔 */
    if ((dt < 1.0e-5f) || (dt > 0.02f)) dt = 1.0f / (float)BMI088_GYRO_ODR_HZ;
    INS_Sample_Dt_Us = BMI088.Gyro_Dt_Us;
#endif

    // 陀螺零漂后台校准: 用本帧刚读到的原始数据累计, 非阻塞(静止约 0.5s 后生效)
    // 参数见 IMU_Cal.h 的 IMU_CAL_GYRO_CAL_*; 也可调 IMU_Cal_Gyro_Bias_Restart() 重来
    IMU_Cal_Gyro_Bias_Update();

    /* 陀螺零漂标定完成的瞬间响一声提示(检测状态机跳到 DONE 的上升沿)。
     * 只设请求, 具体播放在 Buzzer_Task 里做, 这里不会被阻塞; 重新标定成功
     * 也会再响一次(GIVEUP/等待中都不会响) */
    {
        static uint8_t Gyro_Cal_State_Last = IMU_CAL_GYROCAL_IDLE;
        uint8_t Gyro_Cal_State_Now = IMU_Cal_Data.GyroCalState;

        if (Gyro_Cal_State_Now != Gyro_Cal_State_Last)
        {
            if (Gyro_Cal_State_Now == IMU_CAL_GYROCAL_DONE)
                Buzzer_Queue_SoundEffect(Buzzer_SoundEffect_Gyro_Cal_Done);

            Gyro_Cal_State_Last = Gyro_Cal_State_Now;
        }
    }

    /*=== 陀螺滤波: 原始 -> 陷波 -> 二阶低通 ->(可选)一阶低通 ===
     * 滤波全程在“芯片原始轴向”上做; 轴向映射只在最后赋给 INS.*_Speed 时做一次,
     * 这样给 EKF 的那份永远是原始轴向(见 INS_GYRO_FILTER_EKF)。 */
    {
        float gch;

        for (uint8_t i = 0; i < 3; i++)
        {
            gch = INS.Gyro[i] * Rad2Angle;                         /* 原始轴向, rad/s -> °/s */

#if (INS_GYRO_NOTCH_EN == 1u)
            gch = Biquad_Update(&INS_Gyro_Notch[i], gch);          /* 陷波1: 271Hz 低频峰 */
#endif
#if (INS_GYRO_NOTCH2_EN == 1u)
            gch = Biquad_Update(&INS_Gyro_Notch2[i], gch);         /* 陷波2: 657Hz 高频峰 */
#endif
#if (INS_GYRO_LP_EN == 1u)
            gch = Biquad_Update(&INS_Gyro_LP[i], gch);             /* 二阶低通: 压宽带残余 */
#endif

            if (INS_ANG_SPEED_TAU > 0.0f)                          /* 可选一阶低通 */
            {
                float alpha = dt / (INS_ANG_SPEED_TAU + dt);
                INS_Speed_LP[i] += (gch - INS_Speed_LP[i]) * alpha;
            }
            else
            {
                INS_Speed_LP[i] = gch;
            }
        }

#if (INS_GYRO_FILTER_EKF == 1u)
        /* 给 EKF 用: 滤波后角速度, 保持芯片原始轴向(rad/s), 不参与下面的符号翻转 */
        INS_Gyro_F[X] = INS_Speed_LP[0] * Angle2Rad;
        INS_Gyro_F[Y] = INS_Speed_LP[1] * Angle2Rad;
        INS_Gyro_F[Z] = INS_Speed_LP[2] * Angle2Rad;
#endif

        /* 给控制内环/遥测用: 输出坐标系 -> 机体系
         * ★ 2026-10-19 换了新校准 Frame_R(相对旧的绕 Z 转了 90°), 这条映射跟着换:
         *   真实 Roll  角速度 = +GyroX
         *   真实 Pitch 角速度 = -GyroY
         *   真实 Yaw   角速度 = +GyroZ */
        INS.Roll_Speed  =  INS_Speed_LP[0];
        INS.Pitch_Speed = -INS_Speed_LP[1];
        INS.Yaw_Speed   =  INS_Speed_LP[2];
    }

    // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
    INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * Rad2Angle;
    INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * Rad2Angle;

    // 核心函数,EKF更新四元数
#if (INS_GYRO_FILTER_EKF == 1u)
    IMU_QuaternionEKF_Update(INS_Gyro_F[X], INS_Gyro_F[Y], INS_Gyro_F[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);
#else
    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);
#endif

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    {
        float gravity_b[3];

        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
    }

    // 获取最终数据
    INS.Yaw = QEKF_INS.Yaw;
    /* ★ 新 Frame_R 下 QEKF 的 Roll/Pitch 命名正好相反(与上面角速度映射一致):
     *   Roll 取 QEKF.Pitch, Pitch 取 -QEKF.Roll */
    INS.Roll =  QEKF_INS.Pitch;
    INS.Pitch = -QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
    INS.Last_Yaw = INS.Yaw;

    /* 角速度已在上面“陷波->低通”那一步算好(INS.Roll/Pitch/Yaw_Speed),
     * 这里不再重复滤波 */
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
