#include "INS.h"

#include "QuaternionEKF.h"
#include "QMI8658A.h"
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

/*==============================================================================
 *  陀螺滤波链(抑制电机/桨振动): 原始 -> 陷波 -> 二阶低通 ->(可选)一阶低通
 *
 *  为什么上桨后“飘/抖”: QMI8658A 内部只是 ODR/10 ≈ 90Hz 低通, 挡不住
 *  200~450Hz 的桨/电机振动; 这部分能量会被内环 D 项和混控放大,
 *  表现为陀螺读数大幅波动、姿态发抖发飘。
 *  陷波只削那一个窄带, 对环路延迟的影响远小于“一味加大低通”。
 *
 *  ★ INS_GYRO_NOTCH_F0 必须按实测振动主峰改:
 *    1kHz 采一份原始陀螺(建议去滤波, 把 NOTCH/LP 先关掉), 看波形周期或频谱,
 *    取最大那个峰; 常见 2~3 寸桨在 200~400Hz。
 *  若频谱上有不止一个峰, 可以先按最强的那个设 Q 小一点(1~1.5)罩住一片。
 *============================================================================*/
#define INS_FILTER_FS_HZ       1000.0f   /* 滤波器采样率(标称=控制周期 1kHz)   */
#define INS_GYRO_NOTCH_EN      1u        /* 1=陷波使能(主手段)                */
#define INS_GYRO_NOTCH_F0      386.0f    /* 陷波中心频率(Hz) ★按实测改        */
#define INS_GYRO_NOTCH_Q       1.0f      /* Q: 越大越窄; 不确定时取 1~2 宽一点 */
#define INS_GYRO_LP_EN         1u        /* 1=陷波后再加一级二阶低通          */
#define INS_GYRO_LP_FC         150.0f    /* 二阶低通截止频率(Hz)              */
#define INS_GYRO_FILTER_EKF    1u        /* 1=EKF 也用滤波后角速度(默认原始)  */

/* 角速度反馈再串一级一阶低通(兜底): 0=关
 * 注: 0.01(=16Hz) 太滞后, 会让内环发软发飘; 有陷波后这里保持 0 即可 */
#define INS_ANG_SPEED_TAU      0.0f

static float INS_Speed_LP[3] = {0.0f, 0.0f, 0.0f};   /* 3路一阶低通状态 [0]Roll [1]Pitch [2]Yaw */
#if (INS_GYRO_NOTCH_EN == 1u)
static Biquad_t INS_Gyro_Notch[3];
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

    IMU_QuaternionEKF_Init(10, 0.002, 1000000, 1, 0);

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
    }
}



void INS_Task(void)
{
    INS_Init();
    QMI8658A_Init();

    // 上电直接加载固化的校准参数(跳过交互式 IMU_Cal())
    // 需要重新标定时, 把下面 if 块改回 IMU_Cal(); 即可
    if (!IMU_Cal_Load_Config())
    {
        IMU_Cal();
    }

    // 上电陀螺零漂校准: 改为后台运行(见主循环里的 IMU_Cal_Gyro_Bias_Update),
    // 不再在这里阻塞等待静止, 避免上电时被搬动/振动就把任务卡在初始化里。

    for(;;)
    {
        const float gravity[3] = {0, 0, 9.8f};
        dt = DWT_GetDeltaT(&INS_DWT_Count);
        
        // ins update
        IMU_Cal_Get_Calibrated_Data(INS.Accel, INS.Gyro);   // 标定完成后改回此行即可

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
         * 结果给角速度内环(INS.*_Speed)与遥测; 是否给 EKF 见 INS_GYRO_FILTER_EKF。
         * 符号约定: Roll=+GyroX, Pitch=-GyroY, Yaw=+GyroZ(与控制内环一致) */
        {
            static const float Gyro_Sign[3] = { 1.0f, -1.0f, 1.0f };   /* X:+  Y:-  Z:+ */
            float gch;

            for (uint8_t i = 0; i < 3; i++)
            {
                gch = Gyro_Sign[i] * INS.Gyro[i] * Rad2Angle;          /* rad/s -> °/s */

#if (INS_GYRO_NOTCH_EN == 1u)
                gch = Biquad_Update(&INS_Gyro_Notch[i], gch);          /* 陷波: 削振动主峰 */
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

            INS.Roll_Speed  = INS_Speed_LP[0];
            INS.Pitch_Speed = INS_Speed_LP[1];
            INS.Yaw_Speed   = INS_Speed_LP[2];

#if (INS_GYRO_FILTER_EKF == 1u)
            /* 给 EKF 用: 换回 rad/s 与原始轴向(符号取反回去) */
            INS_Gyro_F[X] =  INS.Roll_Speed  * Angle2Rad;
            INS_Gyro_F[Y] = -INS.Pitch_Speed * Angle2Rad;
            INS_Gyro_F[Z] =  INS.Yaw_Speed   * Angle2Rad;
#endif
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
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; i++) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        // 获取最终数据
        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = -QEKF_INS.Roll;
        INS.Roll = QEKF_INS.Pitch;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
        INS.Last_Yaw = INS.Yaw;

        /* 角速度已在上面“陷波->低通”那一步算好(INS.Roll/Pitch/Yaw_Speed),
         * 这里不再重复滤波 */

        osDelay(1);
    }
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
