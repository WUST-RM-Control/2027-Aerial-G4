#ifndef __PARAMETER_H__
#define __PARAMETER_H__

/*==============================================================================
 *  Parameter.h —— 全工程可调参数集中处（调参只改这一个文件）
 *
 *  约定:
 *    1) 这里**只放数值 / 开关**, 不放类型、结构体、函数声明;
 *    2) 各模块原来的 #define 已经搬到这里, 原文件不要再重复定义;
 *    3) 每节都标了单位和"调大/调小的效果", 改完直接全量编译即可。
 *
 *  分节:
 *    1. 系统节拍
 *    2. IMU 滤波 / 姿态解算
 *    3. IMU 校准（六面标定 / 上电零漂）
 *    4. 传感器驱动与滤波（BMI088 / BMP581 / 电池电压 / 气压高度）
 *    5. 飞行控制（功能开关 + 全部 PID）
 *    6. IMU 恒温加热
 *    7. 通信 / 遥测（VOFA）
 *============================================================================*/

/*==============================================================================
 *  1. 系统节拍
 *============================================================================*/
#define FLIGHT_LOOP_HZ            2000u                          /* 飞行主循环(TIM6)频率 */
#define FLIGHT_LOOP_DT            (1.0f / (float)FLIGHT_LOOP_HZ) /* 0.5 ms */

/*==============================================================================
 *  2. IMU 滤波 / 姿态解算  (INS.c)
 *     ★ 陀螺陷波频率是用实测 FFT 定的, 换桨/换机架必须重测
 *============================================================================*/
/* 滤波器采样率 = 陀螺样本率(滤波链每米一个陀螺样本只跑一次, 见 INS.c)。
 * ★ 不要再用 FLIGHT_LOOP_HZ: 主循环与传感器不是一个时钟源, 循环率改高
 *   而 ODR 不变时, 滤波器系数会整体偏移。 */
#define INS_FILTER_FS_HZ          BMI088_GYRO_ODR_HZ   /* 滤波器采样率 = 陀螺 ODR(Hz) */
/* 1 = 用 EXTI 采集的硬件时间戳算真实样本间隔 dt(推荐, 积分时间无积累误差)
 * 0 = 回旧行为: 每个循环节拍都用固定循环 dt 推进一次 EKF */
#define INS_SAMPLE_TS_EN          1u
#define INS_GYRO_NOTCH_EN         1u        /* 1=陷波1(低频峰)                    */
#define INS_GYRO_NOTCH_F0         271.0f    /* 陷波1 中心频率(Hz)  ★实测         */
#define INS_GYRO_NOTCH_Q          6.8f      /* 带宽 = F0/Q ≈ 40 Hz                */
#define INS_GYRO_NOTCH2_EN        1u        /* 1=陷波2(高频峰)                    */
#define INS_GYRO_NOTCH2_F0        657.0f    /* 陷波2 中心频率(Hz)  ★实测         */
#define INS_GYRO_NOTCH2_Q         11.0f     /* 带宽 = F0/Q ≈ 60 Hz                */
#define INS_GYRO_LP_EN            1u        /* 1=陷波后加一级二阶低通              */
#define INS_GYRO_LP_FC            350.0f    /* 二阶低通截止(Hz), 延迟≈0.64 ms     */
#define INS_GYRO_FILTER_EKF       1u        /* 1=EKF 用滤波后角速度, 0=用原始值    */
#define INS_ANG_SPEED_TAU         0.0f      /* 角速度反馈附加一阶低通(s), 0=关     */
#define INS_EKF_ACCEL_LPF_TAU     0.010f    /* EKF 观测(加速度)低通(s), 0=不过滤  */

/* EKF 初始化参数: IMU_QuaternionEKF_Init(过程噪声1, 过程噪声2, 观测噪声, lambda, lpf) */
#define QEKF_PROCESS_NOISE1       10.0f     /* 陀螺过程噪声, 调大更信陀螺         */
#define QEKF_PROCESS_NOISE2       0.005f    /* 零偏过程噪声                       */
#define QEKF_MEASURE_NOISE        1000000.0f/* 加计观测噪声, 调大更信陀螺         */
#define QEKF_LAMBDA               1.0f      /* 渐消记忆因子, 1=标准卡尔曼         */

/*==============================================================================
 *  3. IMU 校准 (IMU_Cal)  —— 交互式六面标定 + 上电后台零漂标定
 *============================================================================*/
#define IMU_CAL_USE_DOUBLE        1u        /* 0=float(快) 1=double(更精确更慢)   */
#define IMU_CAL_DEBUG_EN          0u        /* 1=打印标定引导信息                 */

/* 六面标定姿态采集 */
#define IMU_CAL_POSE_MIN          6u        /* 最少姿态数                         */
#define IMU_CAL_POSE_MAX          12u       /* 姿态数上限(达到即强制完成)         */
#define IMU_CAL_POSE_SEP_COS      0.9511f   /* cos(18°): 两姿态最小夹角            */
#define IMU_CAL_STATIC_SETTLE_MS  500u      /* 判静止后的稳定等待(ms)             */
#define IMU_CAL_ACCUM_MS          1500u     /* 单个姿态的采样平均时长(ms)         */
#define IMU_CAL_MOVE_RAD_S        0.3f      /* 陀螺角速度低于此视为静止(rad/s)    */
#define IMU_CAL_ACC_NORM_MIN      8.0f      /* 静止 |加速度| 下限(m/s²)           */
#define IMU_CAL_ACC_NORM_MAX      11.5f     /* 静止 |加速度| 上限(m/s²)           */
#define IMU_CAL_G_REF             9.8f      /* 参考重力(m/s²)                     */
#define IMU_CAL_RESID_TOL         0.5f      /* 各姿态模长最大离散容差             */

/* 上电后台陀螺零漂标定 */
#define IMU_CAL_GYRO_CAL_EN            1u    /* 1=上电自动实测零漂(解锁前提)      */
#define IMU_CAL_GYRO_CAL_SAMPLES       1000u /* 一批静止样本数(2 kHz → 0.5 s)    */
#define IMU_CAL_GYRO_CAL_SETTLE_MS     200u  /* 判静止后先丢弃的过渡(ms)          */
#define IMU_CAL_GYRO_CAL_BUDGET_MS     5000u /* 总时间预算, 超时放弃(ms)          */
#define IMU_CAL_GYRO_CAL_RETRY_MAX     5u    /* 质检不过最多重试几批              */
#define IMU_CAL_GYRO_CAL_MOVE_RAD_S    0.3f  /* 任一角速度超此值认为在动(rad/s)   */
#define IMU_CAL_GYRO_CAL_MAX_STD       0.1f  /* 每轴标准差上限(rad/s)             */
#define IMU_CAL_GYRO_CAL_MAX_DEV       0.3f  /* 与固化零漂最大合理偏差(rad/s)     */
#define IMU_CAL_GYRO_CAL_AUTO_RETRY_EN 1u    /* 放弃后自动重试                    */
#define IMU_CAL_GYRO_CAL_RETRY_PERIOD_MS 1000u /* 自动重试间隔(ms)                */

/* 交互标定的蜂鸣器进度提示(串口静默时唯一的反馈) */
#define IMU_CAL_BUZZER_EN              1u     /* 1=六个面/各阶段都有专属提示音      */

/*==============================================================================
 *  4. 传感器驱动与滤波
 *============================================================================*/
/* ---- BMI088 六轴 IMU (SPI2) ----
 * ★ 量程是成对配置: "换算系数" 与 "寄存器值" 必须一致, 否则数据整体缩/放大
 *   加速度 ACC_RANGE(0x41): 0x00=±3g  0x01=±6g  0x02=±12g  0x03=±24g
 *   陀   螺 GYRO_RANGE(0x0F): 0x00=±2000dps(最大) 0x01=±1000  0x02=±500  0x03=±250  0x04=±125
 *   当前: 加速度 ±12g / 陀螺 ±2000dps(=芯片最大量程, 已封顶)
 *   加速度量程改大: 能测更大的冲击, 但分辨率变粗(±6g→±12g 每 LSB 从 0.18mg 变 0.37mg),
 *   姿态解算靠重力分量, 一般 ±6g 就够; 只有频繁大加速度(高速机动/碰撞)才需要 ±12g 以上。 */
#define BMI088_ACCEL_RANGE_G      12.0f     /* 加速度量程(±g), 用于换算系数              */
#define BMI088_ACC_RANGE_REG      0x02u     /* 加速度量程寄存器值(0x02 = ±12g)          */
#define BMI088_GYRO_RANGE_DPS     2000.0f   /* 陀螺量程(±dps), ±2000 已是 BMI088 最大     */
#define BMI088_GYRO_RANGE_REG     0x00u     /* 陀螺量程寄存器值(0x00 = ±2000dps)        */
#define BMI088_ID_RETRY           50u       /* 芯片 ID 识别重试次数(× 5 ms)             */
#define BMI088_INT_TIMEOUT_MS     20u       /* INT 不来时的兜底读周期(ms)               */

/* ---- BMI088 采样率(ODR): 已是芯片硬件上限 ----
 *   加速度 ACC_CONF(0x40) 低4位: 0x05=12.5 0x06=25 0x07=50 0x08=100
 *                                0x09=200 0x0A=400 0x0B=800 0x0C=1600Hz(=最大)
 *   陀   螺 GYRO_BANDWIDTH(0x10) 低4位: 0x80|0x00 = 2000Hz/532Hz低通
 *                                      0x80|0x01 = 2000Hz/230Hz低通(=最大ODR)
 *                                      0x80|0x02 = 1000Hz/116Hz  0x80|0x03 = 400Hz/47Hz
 * ★ 再想更快只能换传感器(例如 ICM42688 类 32kHz): BMI088 不接受更高 ODR。
 *   对精度真正有影响的是下面的“样本时间戳”: 用真实样本间隔积分, 而不是假设固定 dt。 */
#define BMI088_ACC_CONF_REG       0xACu     /* 0x80必置|bwp=normal(0x20)|ODR 1600Hz(0x0C) */
#define BMI088_GYRO_BW_REG        0x81u     /* 0x80必置|ODR 2000Hz / 230Hz 低通             */
#define BMI088_GYRO_ODR_HZ        2000u     /* 陀螺实际 ODR(Hz), 上面寄存器的含义            */
#define BMI088_ACC_ODR_HZ         1600u     /* 加速度实际 ODR(Hz)                            */

/* ---- BMP581 气压计 (SPI1) ---- */
#define BMP581_SEA_PRESSURE       101325.0f /* 标准海平面气压(Pa), 测高基准             */
#define BMP581_PRESS_MIN          30000.0f  /* 有效气压下限(Pa)                         */
#define BMP581_PRESS_MAX          125000.0f /* 有效气压上限(Pa)                         */
#define BMP581_ID_RETRY           50u       /* 芯片 ID 识别重试次数(× 5 ms)             */
#define BMP581_INT_TIMEOUT_MS     100u      /* INT 不来时的兜底读周期(ms)               */

/* ---- Sensor 任务滤波 ---- */
#define SENSOR_TAU_VBAT           0.5f      /* 电池电压一阶低通(s)                      */
#define SENSOR_VBAT_VALID_MIN     1.5f      /* 低于此视为无效(未接电池/未转换完)(V)     */
#define SENSOR_TAU_BAR            0.3f      /* 气压高度一阶低通(s)                      */
#define SENSOR_TAU_BAR_SPEED      0.2f      /* 气压速度一阶低通(s)                      */
#define SENSOR_FAN_K              13.44f    /* 高度随油门补偿系数(mm/1% 占空比)         */

/*==============================================================================
 *  5. 飞行控制 (Control.c)
 *============================================================================*/
/* ---- 功能开关 ---- */
#define CONTROL_USE_QUATERNION    1u        /* 1=四元数姿态误差  0=欧拉角相减   */
#define CONTROL_THROTTLE_SQUARE   1u        /* 1=杆量→占空比     0=杆量→推力   */
#define CONTROL_TILT_COMP         1         /* 倾斜油门补偿                     */
#define CONTROL_V_COMP            1         /* 电池电压补偿                     */
#define CONTROL_AUTO_ROLL         0         /* 一键滚转(SD 触发)                */

/* ---- PID 参数 ----  PID_Init(结构体, Kp, Ki, Kd, Kf, I限幅, 输出限幅)
 *      调参顺序: 角速度内环 P → D → FF → I, 再角度外环 P, 最后高度环
 *      单位见每组的注释; 标 MAX 的是输出限幅, IMAX 是积分限幅
 *
 *      Roll / Pitch 对称, 共用一组参数(RP); 如果将来机架不对称需要单独调,
 *      再拆成 AXIS_ROLL / AXIS_PITCH 两组即可。
 */

/* 高度环: 高度误差(mm) -> 速度期望(mm/s) */
#define CTRL_HEIGHT_P_KP          1.0f
#define CTRL_HEIGHT_P_KI          0.0f
#define CTRL_HEIGHT_P_KD          0.01f
#define CTRL_HEIGHT_P_KF          0.0f
#define CTRL_HEIGHT_P_IMAX        0.0f
#define CTRL_HEIGHT_P_MAX         500.0f

/* 高度环: 速度误差(mm/s) -> 推力(%) */
#define CTRL_HEIGHT_S_KP          0.015f
#define CTRL_HEIGHT_S_KI          0.02f
#define CTRL_HEIGHT_S_KD          0.0f
#define CTRL_HEIGHT_S_KF          0.0f
#define CTRL_HEIGHT_S_IMAX        5.0f
#define CTRL_HEIGHT_S_MAX         20.0f

/* 角度外环 Roll+Pitch(deg 误差 -> deg/s), 只用 P */
#define CTRL_ANGLE_RP_KP          15.0f
#define CTRL_ANGLE_RP_KI          0.0f
#define CTRL_ANGLE_RP_KD          0.02f
#define CTRL_ANGLE_RP_KF          0.0f
#define CTRL_ANGLE_RP_IMAX        0.0f
#define CTRL_ANGLE_RP_MAX         500.0f

/* 角度外环 Yaw */
#define CTRL_ANGLE_YAW_KP         20.0f
#define CTRL_ANGLE_YAW_KI         0.0f
#define CTRL_ANGLE_YAW_KD         0.0f
#define CTRL_ANGLE_YAW_KF         0.0f
#define CTRL_ANGLE_YAW_IMAX       0.0f
#define CTRL_ANGLE_YAW_MAX        500.0f

/* 角速度内环 Roll+Pitch(deg/s -> 推力差动) */
#define CTRL_RATE_RP_KP           0.2f
#define CTRL_RATE_RP_KI           0.0f
#define CTRL_RATE_RP_KD           0.0f
#define CTRL_RATE_RP_KF           0.0f
#define CTRL_RATE_RP_IMAX         10.0f
#define CTRL_RATE_RP_MAX          50.0f

/* 角速度内环 Yaw */
#define CTRL_RATE_YAW_KP          0.1f
#define CTRL_RATE_YAW_KI          0.0f
#define CTRL_RATE_YAW_KD          0.0f
#define CTRL_RATE_YAW_KF          0.0f
#define CTRL_RATE_YAW_IMAX        5.0f
#define CTRL_RATE_YAW_MAX         40.0f

/* Roll/Pitch/Yaw 角速度内环 D 项低通截止频率(Hz), 0=关闭 D */
#define CTRL_RATE_D_LPF_HZ        30.0f

/* 定点速度环 X(m/s -> 姿态角) */
#define CTRL_VEL_X_KP             0.03f
#define CTRL_VEL_X_KI             0.05f
#define CTRL_VEL_X_KD             0.0f
#define CTRL_VEL_X_KF             0.0f
#define CTRL_VEL_X_IMAX           50.0f
#define CTRL_VEL_X_MAX            100.0f

/* 定点速度环 Y */
#define CTRL_VEL_Y_KP             0.03f
#define CTRL_VEL_Y_KI             0.05f
#define CTRL_VEL_Y_KD             0.0f
#define CTRL_VEL_Y_KF             0.0f
#define CTRL_VEL_Y_IMAX           50.0f
#define CTRL_VEL_Y_MAX            100.0f

/* PID 算法层默认值 (PID.c) */
#define PID_DT_DEFAULT            0.0005f   /* dt 异常时的兜底控制周期(s)       */
#define PID_D_LPF_DEFAULT         100.0f    /* D 项默认低通截止(Hz), 0=关闭 D   */

/*==============================================================================
 *  6. IMU 恒温加热 (Heater.c)
 *     ★ 所有保护阈值都在这; 改大 TRIP_C 会降低安全性, 谨慎
 *============================================================================*/
/* 温度 / 保护 */
#define HEATER_TARGET_C           40.0f     /* 目标温度(℃)                      */
#define HEATER_SOFT_STOP_C        41.0f     /* 高于此值不加热(不锁存)(℃)         */
#define HEATER_TRIP_C             46.0f     /* 过温跳闸: 锁存(℃)                 */
#define HEATER_TEMP_MIN_VALID     (-40.0f)  /* 有效温度下限(℃)                  */
#define HEATER_TEMP_MAX_VALID     85.0f     /* 有效温度上限 = BMI088 量程(℃)    */
#define HEATER_TEMP_TIMEOUT_MS    100u      /* 温度多久没更新判为无效(ms)         */
#define HEATER_DEADBAND_C         0.3f      /* 高于目标这么多就不再加热(℃)       */

/* PI + 前馈 */
#define HEATER_FF_DUTY            20.0f     /* 前馈占空比(只在低于目标时加)(%)   */
#define HEATER_KP                 25.0f     /* 比例增益(%/℃)                    */
#define HEATER_KI                 0.5f      /* 积分增益(%/(℃·s))                */
#define HEATER_DUTY_MAX           100.0f    /* 占空比上限(%)                    */
#define HEATER_DUTY_SLEW          10.0f     /* 每窗口最大变化量(%)              */

/* 无响应保护: 接近满功率持续这么久, 温度却几乎不涨 -> 锁存 */
#define HEATER_NO_RESPONSE_S      180.0f    /* 持续时间(s)                      */
#define HEATER_NO_RESPONSE_DUTY   80.0f     /* 认为"满功率"的占空比门限(%)       */
#define HEATER_NO_RESPONSE_DT     1.0f      /* 至少涨这么多才算有响应(℃)         */

/* 窗口 / 定时器(与 TIM4 硬件配置强相关, 一般不用改) */
#define HEATER_TIM_WINDOW_MS      20u       /* 控制窗口(ms) = 1MHz × 20000       */
#define HEATER_ARR                19999u    /* TIM4 ARR (1 MHz 计数)            */

/*==============================================================================
 *  7. 通信 / 遥测 (VOFA, USART2)
 *============================================================================*/
#define VOFA_SEND_HZ              1000u     /* 发送频率(Hz), TIM7 自动算 PSC/ARR */
#define VOFA_CH_MAX               16        /* 最多 float 通道数(justfloat)      */

#endif /* __PARAMETER_H__ */
