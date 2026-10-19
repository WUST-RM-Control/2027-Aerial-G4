#ifndef __IMU_CAL_H__
#define __IMU_CAL_H__

#include "main.h"

/* ============================ 数学/数值配置 ============================
 * IMU_CAL_USE_DOUBLE=1: 拟合/旋转用 double(双精度, 精度优先);
 * =0: 用 float(单精度FPU, 更快)。实时/逐帧路径始终为 float。
 * ====================================================================== */
#define IMU_CAL_USE_DOUBLE   1u   /* 0=float(快, 够用); 1=double(更精更慢) */

#if (IMU_CAL_USE_DOUBLE == 1u)
    #define CAL_REAL    double
    #define CAL_ABS(x)  fabs(x)
    #define CAL_SQRT(x) sqrt(x)
#else
    #define CAL_REAL    float
    #define CAL_ABS(x)  fabsf(x)
    #define CAL_SQRT(x) IMU_Cal_Sqrt_F32(x)
#endif

/* 1) 日志/调试输出通道
 * 注意(本工程实测): 工程用 ARM Compiler 6 + MicroLIB 且没有重定向 fputc,
 *   printf 被链接成了空实现 —— 输出被丢弃(不会卡死、不会崩, 但也看不到)。
 * 想看零漂校准结果:
 *   (a) 调试器直接看 IMU_Cal_Data.GyroCalBias / GyroCalStd / GyroCalState;
 *   (b) 或把下面改成 USB_Printf(...) 走 USB CDC —— 但必须等
 *       MX_USB_Device_Init() 执行之后才能调用, 否则 CDC_Transmit_FS 会
 *       解引用 pClassData == NULL 直接 HardFault(它没有做空指针检查)。 */
#define IMU_CAL_DEBUG_EN    0u   /* 1=打印过程调试/操作引导信息(校准需人机交互, 建议开); 0=关闭 */
#define IMU_CAL_PRINTF(...)     printf(__VA_ARGS__)  /* 底层打印: 见上面说明 */
#if (IMU_CAL_DEBUG_EN)
    #define IMU_CAL_LOG(...)    IMU_CAL_PRINTF(__VA_ARGS__)  /* 过程信息(可关) */
#else
    #define IMU_CAL_LOG(...)
#endif

/*
 * IMU 全校准模块
 *  - 加速度计: 全自动采集若干个互不相同的静止姿态, 用最小二乘拟合
 *    逐轴零偏与比例(对角模型);
 *  - 陀螺仪  : 用所有静止样本的均值作为零漂;
 *  - 输出坐标对齐: 校准结束后额外采集一次“模块正面朝上、绝对水平静止”
 *    的姿态, 用它把输出坐标系的 +z 锁定为竖直朝上 —— 因此当模块正面
 *    朝上静止时, 输出加速度严格为 (0, 0, +g)。
 *
 * 使用方式(一次性, 阻塞执行):
 *   1) 在任务开头(循环之外)调用一次 IMU_Cal();  它内部自动读传感器、
 *      采集姿态、拟合、锁定正面参考面, 校准完成后把参数写入
 *      IMU_Cal_Data.Config 并返回;
 *   2) 之后在自己的循环里反复调用
 *      IMU_Cal_Get_Calibrated_Data(&acc,&gyr) 获取校准后的数据
 *      (该函数每次都会内部读一次原始传感器)。
 * 调试信息开关/打印通道等见 IMU_Cal.c 顶部“迁移接口”。
 * 结果保存在 IMU_Cal_Data.Config, 校准过程中的中间量也统一放在
 * IMU_Cal_Data 内(便于查看与管理)。
 * 单位: 加速度 = m/s^2, 角速度 = rad/s(与 BMI088_Read 输出一致)。
 */

/* ------------------------- 可调参数 ------------------------- */
#define IMU_CAL_POSE_MIN           6u      /* 最少姿态数(一般六个面即可)   */
#define IMU_CAL_POSE_MAX           12u     /* 姿态数上限, 达到后强制完成       */
#define IMU_CAL_POSE_SEP_COS       0.9511f /* cos(18°): 两个姿态之间的最小夹角 */
#define IMU_CAL_STATIC_SETTLE_MS   500u    /* 判定静止后的稳定等待时长         */
#define IMU_CAL_ACCUM_MS           1500u    /* 单个姿态的采样平均时长           */
#define IMU_CAL_MOVE_RAD_S         0.3f   /* 陀螺角速度小于该值视为静止       */
#define IMU_CAL_ACC_NORM_MIN       8.0f    /* 静止时 |加速度| 的范围下限(m/s^2)*/
#define IMU_CAL_ACC_NORM_MAX       11.5f   /* 静止时 |加速度| 的范围上限(m/s^2)*/
#define IMU_CAL_G_REF              9.8f    /* 参考重力加速度, 用于归一化尺度   */
#define IMU_CAL_RESID_TOL          0.5f    /* 各姿态校准模长的最大离散容差     */

/* ------------------- 陀螺零漂校准(后台运行, 不阻塞) -------------------
 * 为什么: 陀螺零漂随温度/上电时刻而变化, 代码里固化的 Gyro_Offset 只是
 *   标定那一刻的快照。零漂直接进姿态角积分, 0.01 rad/s 就是约 0.6°/s
 *   的 Yaw 漂移, 所以上电后重新实测一次很有价值。
 * 怎么做: 不在任务里阻塞等静止(那样上电时被搬动就会把任务卡在初始化
 *   里, 而且开机第一帧 dt 会异常大), 而是挂在 INS 主循环里后台跑:
 *   每帧只做 O(1) 累加, 无 while/delay/打印。采满 SAMPLES 帧静止样本
 *   后求均值, 质检通过则覆盖 Config.Gyro_Offset; 一检测到转动就整批
 *   作废重来。
 * 放弃条件: 超过 BUDGET_MS 或质检不过 RETRY_MAX 批 -> 保留原零漂,
 *   本次上电不再尝试(可调 IMU_Cal_Gyro_Bias_Restart() 重新开始)。
 * 注意: MOVE_RAD_S 越小越严。缓慢匀速转动产生的方差很小(标准差查不
 *   出来), 却会给均值带来转速偏置, 所以这个门限要保持小。
 * ------------------------------------------------------------------ */
#define IMU_CAL_GYRO_CAL_EN         1u      /* 1=上电后台自动校准陀螺零漂            */
#define IMU_CAL_GYRO_CAL_SAMPLES    1000u   /* 一批静止样本数(2kHz 循环 → 约0.5s) */
#define IMU_CAL_GYRO_CAL_SETTLE_MS  200u    /* 判定静止后先丢弃的过渡时间(ms)        */
#define IMU_CAL_GYRO_CAL_BUDGET_MS  5000u   /* 总时间预算: 超时则放弃(ms)            */
#define IMU_CAL_GYRO_CAL_RETRY_MAX  5u      /* 质检不过最多重试几批                 */
#define IMU_CAL_GYRO_CAL_MOVE_RAD_S 0.3f   /* 任一轴角速度超此值即认为在动(rad/s)   */
#define IMU_CAL_GYRO_CAL_MAX_STD    0.1f   /* 每轴标准差上限(rad/s), 超了认为抖动   */
#define IMU_CAL_GYRO_CAL_MAX_DEV    0.3f    /* 与固化零漂的最大合理偏差(rad/s, ≈11dps)*/
#define IMU_CAL_GYRO_CAL_AUTO_RETRY_EN   1u /* 放弃后是否隔一段自动再试(非阻塞)      */
#define IMU_CAL_GYRO_CAL_RETRY_PERIOD_MS 1000u /* 自动重试间隔(ms)                    */

/* 总体运行状态 */
typedef enum
{
    IMU_CAL_STATE_ACQUIRE = 0,  /* 阶段1: 采集姿态 */
    IMU_CAL_STATE_REF_UP,       /* 阶段2: 等待“正面朝上”, 锁定输出 +z */
    IMU_CAL_STATE_REF_FWD,      /* 阶段3: 等待 Pitch ~90°, 锁定输出前向 x/y */
    IMU_CAL_STATE_DONE          /* 完成: 校准结束, IMU_Cal() 返回 */
} IMU_Cal_State_e;

/* 采样状态机相位 */
typedef enum
{
    IMU_CAL_PHASE_WAIT_STATIC = 0,  /* 等待模块静止                         */
    IMU_CAL_PHASE_SETTLE,           /* 已静止, 等待 SETTLE_MS 稳定          */
    IMU_CAL_PHASE_ACCUM,            /* 对当前静止姿态取平均                 */
    IMU_CAL_PHASE_WAIT_MOVE         /* 本次已记录, 等待翻转后再采下一个     */
} IMU_Cal_Phase_e;

/* 后台陀螺零漂校准 状态机 */
typedef enum
{
    IMU_CAL_GYROCAL_IDLE = 0,   /* 未开始(上电后首次 Update 自动开始)   */
    IMU_CAL_GYROCAL_WAIT,       /* 等待静止 + 丢弃 SETTLE_MS 过渡样本    */
    IMU_CAL_GYROCAL_ACCUM,      /* 正在累计静止样本                     */
    IMU_CAL_GYROCAL_DONE,       /* 完成: 实测值已覆盖 Gyro_Offset        */
    IMU_CAL_GYROCAL_GIVEUP      /* 放弃: 沿用原零漂, 本次上电不再尝试    */
} IMU_Cal_GyroCal_State_e;

/* 校准参数, 保存在 IMU_Cal_Data.Config */
typedef __PACKED_STRUCT
{
    float   Acc_Offset[3];  /* 加速度计零偏, 单位 m/s^2, 芯片坐标系         */
    float   Acc_Scale[3];   /* 加速度计比例, 芯片坐标系                     */
    float   Gyro_Offset[3]; /* 陀螺仪零漂,   单位 rad/s, 芯片坐标系         */
    float   Frame_R[3][3];  /* 芯片->模块(输出)旋转矩阵 R; 模块正面朝上时
                               输出加速度 = R * (校准后芯片加速度) = (0,0,+g) */
    uint8_t Is_Calibrated;  /* 0 = 未校准/校准中, 1 = 已校准                 */
} IMU_Cal_Config_StructTypedef;

typedef __PACKED_STRUCT
{
    IMU_Cal_Config_StructTypedef Config;    /* 保存的数据(校准参数) */

    /* ---------------- 校准过程中间量(便于查看/调试) ---------------- */
    uint8_t  CalState;      /* 状态机: 0采集 1朝上 2Pitch90 3完成         */
    uint8_t  CalPhase;      /* 采样状态机相位(见 IMU_Cal.c 枚举)            */
    uint32_t PhaseStartMs;  /* 进入当前相位的时刻                          */
    uint8_t  PoseCount;     /* 已采集的互异姿态数量                        */
    float    PoseAcc[IMU_CAL_POSE_MAX][3];  /* 每个姿态的平均原始加速度    */
    float    PoseUnit[IMU_CAL_POSE_MAX][3]; /* 每个姿态的单位向量(判重用)  */
    float    AccSum[3];     /* 当前姿态的加速度累加和                      */
    uint32_t AccCnt;        /* 当前姿态的样本数                            */
    float    GyrSum[3];     /* 陀螺零漂的累加和                            */
    uint32_t GyrCnt;        /* 陀螺零漂累计样本数                          */
    float    RefUpRaw[3];   /* “正面朝上”参考姿态的原始加速度均值          */
    float    RefFwdRaw[3];  /* Pitch ~90° 参考姿态的原始加速度均值(定前向) */

    /* ---------------- 陀螺零漂后台校准(结果 + 中间量) ---------------- */
    uint8_t  GyroCalState;    /* 状态机, 见 IMU_Cal_GyroCal_State_e           */
    uint8_t  GyroCalValid;    /* 锁存: 1=本次上电已有“实测出来并生效”的零漂。
                                 只在实测成功时置 1, 重新标定/中间失败都不会
                                 清 0(防飞行中忽然失锁); 只有加载固化值
                                 (IMU_Cal_Load_Config)或复位时才清 0 */
    uint8_t  GyroCalRetry;    /* 质检不过已重试的批数                         */
    uint32_t GyroCalStartMs;  /* 本轮开始时刻; 放弃时改写为放弃时刻(预算/自动重试计时) */
    uint32_t GyroCalSettleMs; /* 最近一次“判定静止”的时刻                      */
    uint32_t GyroCalCount;    /* 当前批次已累计样本数                         */
    float    GyroCalBias[3];  /* 实测零漂(芯片坐标, rad/s)                    */
    float    GyroCalStd[3];   /* 实测每轴标准差(rad/s, 衡量静止程度)           */
} IMU_Cal_Data_StructTypedef;

extern IMU_Cal_Data_StructTypedef IMU_Cal_Data;

/* ============================ 函数声明 ============================ */
/* ---- 对外主接口 ---- */
void    IMU_Cal(void);                                         /* 一次性阻塞校准(任务循环外调用一次) */
void    IMU_Cal_Get_Calibrated_Data(float *Acc, float *Gyro);  /* 读一次原始数据并输出校准值         */
uint8_t IMU_Cal_Is_Calibrated(void);                           /* 是否已完成校准                      */

/* ---- 陀螺零漂校准(后台, 非阻塞) ---- */
void    IMU_Cal_Gyro_Bias_Update(void);    /* 每帧调用一次: 后台累计/质检/生效           */
void    IMU_Cal_Gyro_Bias_Restart(void);   /* 重新开始一轮(例如解锁前确认静止时)         */
uint8_t IMU_Cal_Gyro_Bias_Is_Valid(void);  /* 1=本次上电实测的零漂已生效 !! 可用于禁止起飞条件 */

/* ---- 校准参数 保存/加载 ---- */
uint8_t IMU_Cal_Load_Config(void);                            /* 加载校准参数: 仅从 NVS 读取(1=有记录并已应用)    */
void    IMU_Cal_Save_Config(void);                            /* 保存当前校准参数到 NVS(校准完成后调用)              */
void    IMU_Cal_Reset_Config(void);                           /* 清除当前参数并抹除 NVS 记录, 立即回到未校准         */

/* ---- 平台接口(可在其它模块直接使用/迁移时替换) ---- */
void    IMU_Cal_Read_Raw(float acc[3], float gyr[3]);          /* 读最新原始 acc(m/s2)/gyr(rad/s) */
float   IMU_Cal_Sqrt_F32(float v);                             /* 单精度平方根(arm_math/FPU)      */

/* ---- 采样/采集相关 ---- */
uint8_t IMU_Cal_Is_Static(const float *acc, const float *gyr); /* 判断是否静止                      */
void    IMU_Cal_Accum_Gyro(const float *gyr);                  /* 静止样本累计陀螺(零漂用)          */
uint8_t IMU_Cal_Sample_One(const float *acc, const float *gyr,
                           uint8_t isStatic, uint32_t now, float mean[3]); /* 采样一个静止姿态平均 */
uint8_t IMU_Cal_Pose_Is_New(const float mean[3]);              /* 新姿态去重判断                     */
void    IMU_Cal_Pose_Add(const float mean[3]);                 /* 记录一个姿态                       */

/* ---- 拟合/输出坐标 ---- */
uint8_t IMU_Cal_GaussSolve(const CAL_REAL *N, const CAL_REAL *rhs, CAL_REAL *x, int n); /* 通用线性求解 */
uint8_t IMU_Cal_Fit_Accel(void);                               /* 两步法拟合 acc 零偏/比例(写Config) */
uint8_t IMU_Cal_Compute_Frame(void);                           /* 由正面参考构造旋转矩阵 Frame_R     */
uint8_t IMU_Cal_Try_Fit(void);                                 /* 姿态够则尝试拟合, 返回是否成功     */
void    IMU_Cal_Finish(void);                                  /* 收尾: 零漂/标志/打印校准值         */

#endif