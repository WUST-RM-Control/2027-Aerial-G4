/*
 * IMU 全校准(全自动 + 输出坐标对齐)
 * ------------------------------------------------------------------
 * 阶段1(姿态采集): 模块静止时采集该姿态的平均原始加速度, 共需要
 *          IMU_CAL_POSE_MIN ~ IMU_CAL_POSE_MAX 个互不相同的静止姿态
 *          (两两夹角 >= 18°)。用“两步法”最小二乘(对角比例 + 零偏,
 *          见 IMU_Cal_Fit_Accel)拟合得到 Acc_Offset / Acc_Scale,
 *          使任意静止姿态下 |校准后加速度| = g。
 * 阶段2(参考面锁定): 提示把模块“正面朝上”放到绝对水平面静止, 采集
 *          该姿态原始加速度 -> 用 Acc_Offset/Acc_Scale 校正得到芯片
 *          坐标系下的“竖直朝上”单位向量 u, 并构造旋转矩阵 R, 使
 *          R*u = (0,0,1), 即正面朝上时输出加速度 = (0,0,+g)。
 *          说明: 加速度计无法感知绕竖直轴的偏航, 因此输出 x/y 取
 *          芯片上垂直于 u 的方向(选芯片 x 尽量指向输出 +x), 绕竖轴
 *          的最终朝向需在整机端另行对齐。
 * 陀螺仪  : 静止期间陀螺样本均值 -> Gyro_Offset; 角速度也用 R 转到
 *          输出坐标系。
 * 上电零漂: IMU_Cal_Gyro_Bias_Update() 挂在 INS 主循环里后台实测陀螺
 *          零漂并覆盖固化的 Gyro_Offset(零漂随温度/上电时刻漂移, 固化
 *          值只作兜底)。全程非阻塞、每帧 O(1), 且有总时间预算, 因此
 *          不会把任务卡在初始化里; 不静止/噪声过大时自动放弃保留原值。
 * 中间量统一放在 IMU_Cal_Data 内, 便于查看; 参数宏见 IMU_Cal.h。
 * ------------------------------------------------------------------
 */
 #include "IMU_Cal.h"

#include "QMI8658A.h"

/* 全局校准数据: 原始数据 + 校准参数(Config) + 校准过程中间量 */
IMU_Cal_Data_StructTypedef IMU_Cal_Data;

/*    IMU 原始数据读取: 取最新原始数据 acc[3](m/s^2)、gyr[3](rad/s)。
 *    本工程用 BMI088; 换传感器/驱动只改这一个函数即可。 */
void IMU_Cal_Read_Raw(float acc[3], float gyr[3])
{
    uint8_t i;
    QMI8658A_Read_Data();
    for (i = 0; i < 3; i++)
    {
        acc[i] = QMI8658A.Accel[i];
        gyr[i] = QMI8658A.Gyro[i];
    }
}

/* ==================== 校准参数 固化(直接写死) ====================
 * 下面这组参数是从调试器里读到的 IMU_Cal_Data.Config(已完成校准)。
 * 上电时调用 IMU_Cal_Load_Config() 会直接把它们写入 Config,
 * 从而跳过每次都要手动摆姿势的交互式 IMU_Cal() 校准。
 * 若以后改成 Flash 存储, 把 IMU_Cal_Load_Config() 改为读 Flash 即可。
 * ---------------------------------------------------------------- */
static const float IMU_Cal_Fixed_Acc_Offset[3]  = { 0.304593325f,    0.206411183f,   -0.485507458f };   /* m/s^2 */
static const float IMU_Cal_Fixed_Acc_Scale[3]   = { 1.00063789f,     1.00721169f,     0.994780362f  };   /* 比例 */
static const float IMU_Cal_Fixed_Gyro_Offset[3] = {-0.171457797f,    0.0722558275f,  -0.00584641751f }; /* rad/s */
static const float IMU_Cal_Fixed_Frame_R[3][3]  = {
    {-0.0638484657f,  -0.997871995f,   0.0132223275f },
    { 0.997933388f,   -0.0637451038f,  0.00809651893f},
    {-0.00723643089f,  0.0137119517f,  0.999879777f  },
};
/* 加载固化(或将来从 Flash 读回)的校准参数到 IMU_Cal_Data.Config。
 * 返回 1 = 成功(已应用), 0 = 无可用参数。 */
uint8_t IMU_Cal_Load_Config(void)
{
    memcpy(IMU_Cal_Data.Config.Acc_Offset,  IMU_Cal_Fixed_Acc_Offset,  sizeof(IMU_Cal_Data.Config.Acc_Offset));
    memcpy(IMU_Cal_Data.Config.Acc_Scale,   IMU_Cal_Fixed_Acc_Scale,   sizeof(IMU_Cal_Data.Config.Acc_Scale));
    memcpy(IMU_Cal_Data.Config.Gyro_Offset, IMU_Cal_Fixed_Gyro_Offset, sizeof(IMU_Cal_Data.Config.Gyro_Offset));
    memcpy(IMU_Cal_Data.Config.Frame_R,     IMU_Cal_Fixed_Frame_R,     sizeof(IMU_Cal_Data.Config.Frame_R));

    IMU_Cal_Data.Config.Is_Calibrated = 1;
    /* 装进来的是固化值, 不是本次实测值 -> 陀螺零漂仍未“标定好”(禁止起飞),
     * 必须等 IMU_Cal_Gyro_Bias_Update() 实测成功才会置 1 */
    IMU_Cal_Data.GyroCalValid = 0u;

    if(IMU_Cal_Data.Config.Is_Calibrated)  return 1;
    else                                   return 0;
}
   

/* 保存当前校准参数到非易失存储(预留)。
 * 目前参数直接固化在代码里, 尚未实现 Flash 写入。 */
void IMU_Cal_Save_Config(void)
{
    /* TODO: 需要持久化时, 在这里把 IMU_Cal_Data.Config 写入 Flash */
}

/* 清除当前校准参数并抹除保存值: 立即回到“未校准”(原始直通) */
void IMU_Cal_Reset_Config(void)
{
    memset(&IMU_Cal_Data.Config, 0, sizeof(IMU_Cal_Data.Config));   /* Is_Calibrated = 0 */
    IMU_Cal_Data.GyroCalValid = 0u;                                 /* 零漂作废 -> 禁止起飞 */
}

/* 单精度平方根: 供实时/每帧采样路径用, 精度同 sqrtf
 * 移植: 原用 CMSIS-DSP arm_sqrt_f32, ESP32 直接调 libm sqrtf */
float IMU_Cal_Sqrt_F32(float v)
{
    return sqrtf(v);
}

 /* ============================ 公共接口 ============================= */
/*
 * 读校准后数据(模块/输出坐标系)。函数内部会先调用 IMU_Cal_Read_Raw
 * 读一次最新原始数据再换算, 无需外部先喂数据:
 *   cal_chip = (raw - Acc_Offset) .* Acc_Scale      (芯片坐标, m/s^2)
 *   Acc      = Frame_R * cal_chip                    (正面朝上 -> (0,0,+g))
 *   Gyro     = Frame_R * (raw_gyro - Gyro_Offset)
 * 未校准时, 直接输出刚读到的原始数据。
 */
void IMU_Cal_Get_Calibrated_Data(float *Acc, float *Gyro)
{
    uint8_t i, j;
    float rawA[3], rawG[3];
    if (Acc == NULL || Gyro == NULL) return;

    /* 在函数内读一遍最新原始数据 */
    IMU_Cal_Read_Raw(rawA, rawG);

    if (IMU_Cal_Data.Config.Is_Calibrated)
    {
        const float (*R)[3] = IMU_Cal_Data.Config.Frame_R;
        float ca[3], cg[3];
        for (i = 0; i < 3; i++)
        {
            ca[i] = (rawA[i] - IMU_Cal_Data.Config.Acc_Offset[i])
                    * IMU_Cal_Data.Config.Acc_Scale[i];
            cg[i] = rawG[i] - IMU_Cal_Data.Config.Gyro_Offset[i];
        }
        for (i = 0; i < 3; i++)
        {
            float sa = 0.0f, sg = 0.0f;
            for (j = 0; j < 3; j++)
            {
                sa += R[i][j] * ca[j];
                sg += R[i][j] * cg[j];
            }
            Acc[i]  = sa;
            Gyro[i] = sg;
        }
    }
    else
    {
        for (i = 0; i < 3; i++)
        {
            Acc[i]  = rawA[i];
            Gyro[i] = rawG[i];
        }
    }
}

/* 是否已完成校准 */
uint8_t IMU_Cal_Is_Calibrated(void)
{
    return IMU_Cal_Data.Config.Is_Calibrated;
}

/* ==================== 陀螺零漂校准(后台, 非阻塞) ====================
 * 为什么需要: 陀螺零漂随温度/上电时刻缓慢变化, 代码里固化的
 *   Gyro_Offset 只是标定那一刻的快照。零漂直接进入姿态角积分,
 *   哪怕 0.01 rad/s 也会造成约 0.6°/s 的 Yaw 漂移, 所以上电后
 *   重新实测一次很有价值。
 * 为什么不用“阻塞式”: 在任务里 while + osDelay 等静止, 一旦上电时
 *   一直被搬动/振动, 就会把任务卡在初始化里(而且开机后第一帧 dt 会
 *   变得很大, EKF 会被带偏)。所以改成挂在 INS 主循环里的后台流程。
 * 流程(每帧只做 O(1) 工作, 无 while/delay/打印):
 *   [等待静止(SETTLE_MS 过渡)] -> [累计 SAMPLES 帧] -> [求均值/标准差]
 *   -> [质检: 标准差 + 与固化零漂的合理偏差] -> 通过则覆盖 Gyro_Offset。
 * 放弃条件: 超过 BUDGET_MS, 或质检不过 RETRY_MAX 批 -> 标记为“放弃”
 *   (此时 Gyro_Offset 仍是固化值, IMU_Cal_Gyro_Bias_Is_Valid() 返回 0,
 *    上层据此禁止起飞); 隔 AUTO_RETRY 设置的时间会自动再试一次, 不
 *    需要重新上电。
 * 因为每帧都会返回、且总时间有上限, 它不可能卡住调用它的任务。
 * 坐标系: 用的是芯片坐标系原始角速度, 与 Config.Gyro_Offset 一致。
 * ---------------------------------------------------------------- */

/* 一阶和(零漂均值)/二阶和(标准差)。放 static 而不是塞进 packed
 * 结构体里, 避免未对齐的 64bit 访问。 */
static double IMU_Cal_GyroSum[3];
static double IMU_Cal_GyroSum2[3];

/* 重新开始一轮后台零漂校准(结果清空, 从“等待静止”重新计时) */
void IMU_Cal_Gyro_Bias_Restart(void)
{
    uint8_t i;

    IMU_Cal_Data.GyroCalState    = IMU_CAL_GYROCAL_WAIT;
    IMU_Cal_Data.GyroCalRetry    = 0u;
    IMU_Cal_Data.GyroCalCount    = 0u;
    IMU_Cal_Data.GyroCalStartMs  = HAL_GetTick();
    IMU_Cal_Data.GyroCalSettleMs = IMU_Cal_Data.GyroCalStartMs;
    for (i = 0; i < 3; i++)
    {
        IMU_Cal_GyroSum[i]          = 0.0;
        IMU_Cal_GyroSum2[i]         = 0.0;
        IMU_Cal_Data.GyroCalBias[i] = 0.0f;
        IMU_Cal_Data.GyroCalStd[i]  = 0.0f;
    }
}

/*
 * 本次上电的零漂是否已有“实测出来并生效”的值(锁存标志 GyroCalValid)。
 * 只有加载固化值后还没实测出来、或复位过的情况下才返回 0 —— 上层用它
 * 禁止飞机起动(见 Control_Task 的解锁条件)。
 * 注: 后台重开一轮标定不会把已生效的值作废(防飞行中忽然失锁);
 *     IMU_CAL_GYRO_CAL_EN=0 时无人标定, 这里给 1(不做限制, 以宏配置为准)。
 */
uint8_t IMU_Cal_Gyro_Bias_Is_Valid(void)
{
#if (IMU_CAL_GYRO_CAL_EN == 1u)
    return IMU_Cal_Data.GyroCalValid;
#else
    return 1u;
#endif
}

/*
 * 后台零漂校准: 放在 INS 主循环里、IMU_Cal_Get_Calibrated_Data() 之后,
 * 每帧调用一次即可(内部自动开始/自动结束)。
 * 数据来源是本帧已经读好的原始值 QMI8658A.Accel/Gyro(芯片坐标),
 * 所以不会再额外占用 SPI、也不会改变采样节奏。
 */
void IMU_Cal_Gyro_Bias_Update(void)
{
#if (IMU_CAL_GYRO_CAL_EN == 1u)
    const float *acc = QMI8658A.Accel;
    const float *gyr = QMI8658A.Gyro;
    uint32_t now;
    float    norm, mean[3], sd[3];
    uint8_t  i;

    if (IMU_Cal_Data.GyroCalState == IMU_CAL_GYROCAL_DONE) return;   /* 已完成, 不再改动 */

    now = HAL_GetTick();

    /* 上一轮放弃了: 隔 RETRY_PERIOD_MS 自动再试一次(仍然非阻塞, 每帧只判断时间)。
     * 这样上电时被搬动/振动只是“晚一点才好”, 不需要重新上电 */
    if (IMU_Cal_Data.GyroCalState == IMU_CAL_GYROCAL_GIVEUP)
    {
#if (IMU_CAL_GYRO_CAL_AUTO_RETRY_EN == 1u)
        if ((uint32_t)(now - IMU_Cal_Data.GyroCalStartMs) < IMU_CAL_GYRO_CAL_RETRY_PERIOD_MS)
            return;
        IMU_Cal_Gyro_Bias_Restart();            /* 重开一轮(内部会把 StartMs 置当前) */
        now = HAL_GetTick();                    /* 重新取时: 否则上面那 1ms 抖动会误判超预算 */
#else
        return;                                 /* 关闭自动重试: 请用 Restart() 手动重来 */
#endif
    }
    else if (IMU_Cal_Data.GyroCalState == IMU_CAL_GYROCAL_IDLE)
    {
        IMU_Cal_Gyro_Bias_Restart();            /* 上电后第一次调用: 自动开始 */
        now = HAL_GetTick();
    }

    /* --- 总预算: 到点就放弃, 保证本轮一定结束 --- */
    if ((uint32_t)(now - IMU_Cal_Data.GyroCalStartMs) >= IMU_CAL_GYRO_CAL_BUDGET_MS)
    {
        IMU_Cal_Data.GyroCalState   = IMU_CAL_GYROCAL_GIVEUP;
        IMU_Cal_Data.GyroCalStartMs = now;      /* 放弃时刻: 自动重试的间隔从这里算 */
        IMU_CAL_LOG("[GyroCal] give up (board not still) -> retry later\r\n");
        return;
    }

    /* --- 静止判据: 加速度模长接近 g, 且三轴角速度都很小 --- */
    norm = IMU_Cal_Sqrt_F32(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    if (norm < IMU_CAL_ACC_NORM_MIN || norm > IMU_CAL_ACC_NORM_MAX ||
        fabsf(gyr[0]) > IMU_CAL_GYRO_CAL_MOVE_RAD_S ||
        fabsf(gyr[1]) > IMU_CAL_GYRO_CAL_MOVE_RAD_S ||
        fabsf(gyr[2]) > IMU_CAL_GYRO_CAL_MOVE_RAD_S)
    {
        /* 有运动: 本批作废, 回到“等待静止”重新计时 */
        IMU_Cal_Data.GyroCalState    = IMU_CAL_GYROCAL_WAIT;
        IMU_Cal_Data.GyroCalSettleMs = now;
        if (IMU_Cal_Data.GyroCalCount != 0u)
        {
            IMU_Cal_Data.GyroCalCount = 0u;
            for (i = 0; i < 3; i++) { IMU_Cal_GyroSum[i] = 0.0; IMU_Cal_GyroSum2[i] = 0.0; }
        }
        return;
    }

    /* --- 静止: 先丢弃 SETTLE_MS 的过渡样本, 再开始累计 --- */
    if (IMU_Cal_Data.GyroCalState == IMU_CAL_GYROCAL_WAIT)
    {
        if ((uint32_t)(now - IMU_Cal_Data.GyroCalSettleMs) < IMU_CAL_GYRO_CAL_SETTLE_MS)
            return;
        IMU_Cal_Data.GyroCalState = IMU_CAL_GYROCAL_ACCUM;
        IMU_Cal_Data.GyroCalCount = 0u;
        for (i = 0; i < 3; i++) { IMU_Cal_GyroSum[i] = 0.0; IMU_Cal_GyroSum2[i] = 0.0; }
    }

    /* --- 累计本帧样本 --- */
    for (i = 0; i < 3; i++)
    {
        double v = (double)gyr[i];
        IMU_Cal_GyroSum[i]  += v;
        IMU_Cal_GyroSum2[i] += v * v;
    }
    IMU_Cal_Data.GyroCalCount++;

    if (IMU_Cal_Data.GyroCalCount < IMU_CAL_GYRO_CAL_SAMPLES) return;   /* 本批还没采满 */

    /* --- 采满一批: 求均值/标准差并质检 --- */
    {
        uint32_t n   = IMU_Cal_Data.GyroCalCount;
        uint8_t  bad = 0u;

        for (i = 0; i < 3; i++)
        {
            double var;
            mean[i] = (float)(IMU_Cal_GyroSum[i] / (double)n);
            /* 方差 = E[x^2] - E[x]^2 (double 处累计, 避免单精度大幅抵消) */
            var = IMU_Cal_GyroSum2[i] / (double)n - (double)mean[i] * (double)mean[i];
            sd[i] = (var > 0.0) ? IMU_Cal_Sqrt_F32((float)var) : 0.0f;
            if (sd[i] > IMU_CAL_GYRO_CAL_MAX_STD) bad = 1u;   /* 抖动/振动过大 */
        }
        for (i = 0; i < 3; i++)                 /* 无论成败都留下实测值便于调试 */
        {
            IMU_Cal_Data.GyroCalBias[i] = mean[i];
            IMU_Cal_Data.GyroCalStd[i]  = sd[i];
        }

        /* 合理性检查: 与固化零漂偏差过大 -> 上电时被搬动/传感器异常 */
        if (!bad && IMU_Cal_Data.Config.Is_Calibrated)
        {
            for (i = 0; i < 3; i++)
            {
                if (fabsf(mean[i] - IMU_Cal_Data.Config.Gyro_Offset[i]) > IMU_CAL_GYRO_CAL_MAX_DEV)
                    bad = 1u;
            }
        }

        if (!bad)                               /* 质检通过 -> 生效 */
        {
            for (i = 0; i < 3; i++) IMU_Cal_Data.Config.Gyro_Offset[i] = mean[i];
            IMU_Cal_Data.GyroCalState = IMU_CAL_GYROCAL_DONE;
            IMU_Cal_Data.GyroCalValid = 1u;     /* 锁存: 从此允许解锁 */
            IMU_CAL_LOG("[GyroCal] updated (n=%u): %.5f, %.5f, %.5f rad/s\r\n",
                        (uint32_t)n, mean[0], mean[1], mean[2]);
            return;
        }

        /* 质检不过: 重来一批(仍受总预算约束) */
        IMU_Cal_Data.GyroCalRetry++;
        if (IMU_Cal_Data.GyroCalRetry >= IMU_CAL_GYRO_CAL_RETRY_MAX)
        {
            IMU_Cal_Data.GyroCalState   = IMU_CAL_GYROCAL_GIVEUP;
            IMU_Cal_Data.GyroCalStartMs = now;      /* 放弃时刻: 自动重试的间隔从这里算 */
            IMU_CAL_LOG("[GyroCal] give up (noisy) -> retry later\r\n");
            return;
        }
        IMU_Cal_Data.GyroCalState    = IMU_CAL_GYROCAL_WAIT;
        IMU_Cal_Data.GyroCalSettleMs = now;
        IMU_Cal_Data.GyroCalCount    = 0u;
        for (i = 0; i < 3; i++) { IMU_Cal_GyroSum[i] = 0.0; IMU_Cal_GyroSum2[i] = 0.0; }
    }
#endif  /* IMU_CAL_GYRO_CAL_EN */
}

/* ======================= 一次性校准(调用一次) ======================== */
/*
 * 校准过程需要一个人配合:
 *   阶段1: 把模块放在不同面/方向并静止, 自动采集 6~12 个互异姿态;
 *   阶段2: 提示后把模块“正面朝上”放到绝对水平面静止, 锁定输出 +z。
 * 本函数阻塞执行, 直到校准完成(写入 Config 并打印校准参数)才返回。
 * 调用方式: 在任务开头(循环之外)调用一次即可; 之后在自己的循环里反复
 *         调用 IMU_Cal_Get_Calibrated_Data() 获取校准后的数据。
 */
void IMU_Cal(void)
{
    float rawA[3];
    float rawG[3];
    float mean[3];
    uint32_t RefRemindMs;
    uint8_t  finished = 0;
    uint8_t  i;

    /* 从未校准状态开始, 同时清零所有中间量 */
    memset(&IMU_Cal_Data, 0, sizeof(IMU_Cal_Data));
    IMU_Cal_Data.CalState   = IMU_CAL_STATE_ACQUIRE;
    IMU_Cal_Data.CalPhase   = IMU_CAL_PHASE_WAIT_STATIC;
    IMU_Cal_Data.PhaseStartMs = HAL_GetTick();
    RefRemindMs = HAL_GetTick();

    /* 启动提示(ASCII 便于各终端显示) */
    IMU_CAL_LOG("\r\n===============================================\r\n");
    IMU_CAL_LOG("        IMU FULL CALIBRATION (auto)\r\n");
    IMU_CAL_LOG("===============================================\r\n");
    IMU_CAL_LOG("STEP 1) Put the module STILL on any pose, then flip it to\r\n");
    IMU_CAL_LOG("   DIFFERENT poses (>=%u, cover +X/-X/+Y/-Y/+Z/-Z & slanted).\r\n",
                (uint32_t)IMU_CAL_POSE_MIN);
    IMU_CAL_LOG("STEP 2) When asked, put module FRONT face UP on a LEVEL\r\n");
    IMU_CAL_LOG("   surface and keep still (locks output z = vertical up).\r\n");
    IMU_CAL_LOG("STEP 3) When asked, PITCH the module ~90 deg (its FRONT/head\r\n");
    IMU_CAL_LOG("   edge goes DOWN to the table) and keep still. This fixes\r\n");
    IMU_CAL_LOG("   the output x/y = module axes even if the chip is mounted\r\n");
    IMU_CAL_LOG("   rotated/tilted relative to the module (no more yaw coupling).\r\n");

    while (!finished)
    {
        const float *acc;
        const float *gyr;
        uint8_t  isStatic;
        uint32_t now;

        /* 读一次最新原始数据(直接用局部变量) */
        IMU_Cal_Read_Raw(rawA, rawG);
        acc = rawA;
        gyr = rawG;
        isStatic = IMU_Cal_Is_Static(acc, gyr);
        now = HAL_GetTick();

        switch (IMU_Cal_Data.CalState)
        {
        /* ---------- 阶段1: 采集姿态 ---------- */
        case IMU_CAL_STATE_ACQUIRE:
            if (IMU_Cal_Sample_One(acc, gyr, isStatic, now, mean))
            {
                if (IMU_Cal_Pose_Is_New(mean))      /* 是新的姿态才记录 */
                {
                    IMU_Cal_Pose_Add(mean);
                    IMU_CAL_LOG("[Cal] pose %u/%u captured  acc=%.3f,%.3f,%.3f\r\n",
                                (uint32_t)IMU_Cal_Data.PoseCount, (uint32_t)IMU_CAL_POSE_MAX,
                                mean[0], mean[1], mean[2]);
                    if (IMU_Cal_Try_Fit())
                    {
                        /* 拟合成功 -> 进入参考面锁定阶段 */
                        IMU_CAL_LOG("[Cal] accel fitted. Now put module FRONT face\r\n");
                        IMU_CAL_LOG("      UP on a LEVEL surface and keep still...\r\n");
                        IMU_Cal_Data.CalState   = IMU_CAL_STATE_REF_UP;
                        IMU_Cal_Data.CalPhase   = IMU_CAL_PHASE_WAIT_MOVE; /* 需先动一下再静止 */
                        IMU_Cal_Data.PhaseStartMs = now;
                        RefRemindMs = now;
                    }
                    else
                    {
                        IMU_CAL_LOG("[Cal] flip to a different pose and keep still...\r\n");
                    }
                }
                else
                {
                    IMU_CAL_LOG("[Cal] duplicate orientation (%u/%u), flip somewhere else\r\n",
                                (uint32_t)IMU_Cal_Data.PoseCount, (uint32_t)IMU_CAL_POSE_MAX);
                }
            }
            break;

        /* ---------- 阶段2: 锁定“正面朝上”参考面(输出 +z) ---------- */
        case IMU_CAL_STATE_REF_UP:
            if (IMU_Cal_Sample_One(acc, gyr, isStatic, now, mean))
            {
                for (i = 0; i < 3; i++) IMU_Cal_Data.RefUpRaw[i] = mean[i]; /* 参考面原始均值 */
                IMU_CAL_LOG("[Cal] front-up captured (z vertical OK). Now PITCH the\r\n");
                IMU_CAL_LOG("      module ~90 deg so its FRONT/head edge goes DOWN\r\n");
                IMU_CAL_LOG("      to the table, then hold still...\r\n");
                IMU_Cal_Data.CalState   = IMU_CAL_STATE_REF_FWD;
                IMU_Cal_Data.CalPhase   = IMU_CAL_PHASE_WAIT_MOVE; /* 需先动一下再静止 */
                IMU_Cal_Data.PhaseStartMs = now;
                RefRemindMs = now;
            }
            else if ((uint32_t)(now - RefRemindMs) >= 6000u)
            {
                RefRemindMs = now;
                IMU_CAL_LOG("[Cal] waiting: put FRONT face UP on LEVEL surface & still...\r\n");
            }
            break;

        /* ---------- 阶段3: 锁定 Pitch ~90° 前向参考(输出 x/y) ---------- */
        case IMU_CAL_STATE_REF_FWD:
            if (IMU_Cal_Sample_One(acc, gyr, isStatic, now, mean))
            {
                /* 校验: 前向参考应与“竖直朝上”近似垂直(夹角 60°~120°) */
                float nU  = IMU_Cal_Sqrt_F32(IMU_Cal_Data.RefUpRaw[0] * IMU_Cal_Data.RefUpRaw[0] +
                                             IMU_Cal_Data.RefUpRaw[1] * IMU_Cal_Data.RefUpRaw[1] +
                                             IMU_Cal_Data.RefUpRaw[2] * IMU_Cal_Data.RefUpRaw[2]);
                float nM  = IMU_Cal_Sqrt_F32(mean[0] * mean[0] + mean[1] * mean[1] + mean[2] * mean[2]);
                float dot = (nU > 1e-6f && nM > 1e-6f)
                            ? (IMU_Cal_Data.RefUpRaw[0] * mean[0] +
                               IMU_Cal_Data.RefUpRaw[1] * mean[1] +
                               IMU_Cal_Data.RefUpRaw[2] * mean[2]) / (nU * nM)
                            : 1.0f;

                if (fabsf(dot) <= 0.5f)             /* 60°~120°: 可用 */
                {
                    for (i = 0; i < 3; i++) IMU_Cal_Data.RefFwdRaw[i] = mean[i];
                    if (IMU_Cal_Compute_Frame())    /* 用 up + 前向构造完整 R */
                    {
                        IMU_CAL_LOG("[Cal] pitch reference captured -> x/y aligned to module\r\n");
                        IMU_Cal_Finish();
                        finished = 1;
                    }
                    else
                    {
                        IMU_CAL_LOG("[Cal] frame build failed, redo the pitch...\r\n");
                        IMU_Cal_Data.CalPhase   = IMU_CAL_PHASE_WAIT_MOVE;
                        IMU_Cal_Data.PhaseStartMs = now;
                    }
                }
                else
                {
                    IMU_CAL_LOG("[Cal] pitch too small / wrong pose (need ~90 deg).\r\n");
                    IMU_CAL_LOG("      Pitch MORE (front edge down ~90 deg) & hold still.\r\n");
                    IMU_Cal_Data.CalPhase   = IMU_CAL_PHASE_WAIT_MOVE;
                    IMU_Cal_Data.PhaseStartMs = now;
                    RefRemindMs = now;
                }
            }
            else if ((uint32_t)(now - RefRemindMs) >= 6000u)
            {
                RefRemindMs = now;
                IMU_CAL_LOG("[Cal] waiting: PITCH ~90 deg (front edge down) & still...\r\n");
            }
            break;

        case IMU_CAL_STATE_DONE:
        default:
            finished = 1;                           /* 兜底退出 */
            break;
        }

        osDelay(1);                                 /* 让出 CPU, 保持采样节奏 */
    }
}

/* ------------------------- 小工具函数 ----------------------- */
/* 判断单帧是否静止: 角速度足够小 且 加速度模长接近 g */
uint8_t IMU_Cal_Is_Static(const float *acc, const float *gyr)
{
    float norm = IMU_Cal_Sqrt_F32(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    float w    = fabsf(gyr[0]);
    float t    = fabsf(gyr[1]);
    if (t > w) w = t;
    t = fabsf(gyr[2]);
    if (t > w) w = t;

    if (w   >= IMU_CAL_MOVE_RAD_S) return 0;
    if (norm <  IMU_CAL_ACC_NORM_MIN) return 0;
    if (norm >  IMU_CAL_ACC_NORM_MAX) return 0;
    return 1;
}

/* 静止期间累计陀螺样本, 用于计算零漂 */
void IMU_Cal_Accum_Gyro(const float *gyr)
{
    IMU_Cal_Data.GyrSum[0] += gyr[0];
    IMU_Cal_Data.GyrSum[1] += gyr[1];
    IMU_Cal_Data.GyrSum[2] += gyr[2];
    IMU_Cal_Data.GyrCnt++;
}

/*
 * 通用“单次静止姿态采样”状态机(阶段1采集姿态 与 阶段2参考面 共用)。
 * 每次产生一个静止姿态平均后返回 1 并把均值写入 mean[3], 否则返回 0。
 */
uint8_t IMU_Cal_Sample_One(const float *acc, const float *gyr,
                           uint8_t isStatic, uint32_t now, float mean[3])
{
    switch (IMU_Cal_Data.CalPhase)
    {
    case IMU_CAL_PHASE_WAIT_STATIC:
        if (isStatic)                               /* 开始静止 -> 进入稳定等待 */
        {
            IMU_Cal_Data.PhaseStartMs = now;
            IMU_Cal_Data.CalPhase = IMU_CAL_PHASE_SETTLE;
        }
        break;

    case IMU_CAL_PHASE_SETTLE:
        if (!isStatic) { IMU_Cal_Data.CalPhase = IMU_CAL_PHASE_WAIT_STATIC; break; }
        IMU_Cal_Accum_Gyro(gyr);                    /* 静止期累计陀螺(零漂) */
        if ((uint32_t)(now - IMU_Cal_Data.PhaseStartMs) >= IMU_CAL_STATIC_SETTLE_MS)
        {
            /* 稳定时间足够, 开始对该姿态取平均 */
            IMU_Cal_Data.AccSum[0] = acc[0];
            IMU_Cal_Data.AccSum[1] = acc[1];
            IMU_Cal_Data.AccSum[2] = acc[2];
            IMU_Cal_Data.AccCnt = 1;
            IMU_Cal_Data.PhaseStartMs = now;
            IMU_Cal_Data.CalPhase = IMU_CAL_PHASE_ACCUM;
        }
        break;

    case IMU_CAL_PHASE_ACCUM:
        if (!isStatic) { IMU_Cal_Data.CalPhase = IMU_CAL_PHASE_WAIT_STATIC; break; }
        IMU_Cal_Accum_Gyro(gyr);
        IMU_Cal_Data.AccSum[0] += acc[0];
        IMU_Cal_Data.AccSum[1] += acc[1];
        IMU_Cal_Data.AccSum[2] += acc[2];
        IMU_Cal_Data.AccCnt++;
        if ((uint32_t)(now - IMU_Cal_Data.PhaseStartMs) >= IMU_CAL_ACCUM_MS)
        {
            /* 平均时间到, 产出该姿态均值 */
            mean[0] = IMU_Cal_Data.AccSum[0] / (float)IMU_Cal_Data.AccCnt;
            mean[1] = IMU_Cal_Data.AccSum[1] / (float)IMU_Cal_Data.AccCnt;
            mean[2] = IMU_Cal_Data.AccSum[2] / (float)IMU_Cal_Data.AccCnt;
            IMU_Cal_Data.CalPhase = IMU_CAL_PHASE_WAIT_MOVE;
            IMU_Cal_Data.PhaseStartMs = now;
            return 1;
        }
        break;

    case IMU_CAL_PHASE_WAIT_MOVE:
        if (!isStatic)                              /* 检测到翻转 -> 等待下一次静止 */
        {
            IMU_Cal_Data.CalPhase = IMU_CAL_PHASE_WAIT_STATIC;
            IMU_Cal_Data.PhaseStartMs = now;
        }
        else
        {
            IMU_Cal_Accum_Gyro(gyr);                /* 静止期间继续累计零漂 */
        }
        break;

    default:
        IMU_Cal_Data.CalPhase = IMU_CAL_PHASE_WAIT_STATIC;
        break;
    }
    return 0;
}

/* 判断新姿态是否与已采姿态足够不同(夹角 >= 18° 才算新) */
uint8_t IMU_Cal_Pose_Is_New(const float mean[3])
{
    uint8_t i;
    float n = IMU_Cal_Sqrt_F32(mean[0]*mean[0] + mean[1]*mean[1] + mean[2]*mean[2]);
    if (n < 1e-6f) return 0;

    for (i = 0; i < IMU_Cal_Data.PoseCount; i++)
    {
        float dot = (mean[0]*IMU_Cal_Data.PoseUnit[i][0] +
                     mean[1]*IMU_Cal_Data.PoseUnit[i][1] +
                     mean[2]*IMU_Cal_Data.PoseUnit[i][2]) / n;
        if (dot >= IMU_CAL_POSE_SEP_COS) return 0;  /* 与已存姿态太接近 */
    }
    return 1;
}

/* 记录一个新姿态(原始向量 + 单位向量) */
void IMU_Cal_Pose_Add(const float mean[3])
{
    uint8_t i;
    float n;
    if (IMU_Cal_Data.PoseCount >= IMU_CAL_POSE_MAX) return;   /* 数组已满防越界 */
    n = IMU_Cal_Sqrt_F32(mean[0]*mean[0] + mean[1]*mean[1] + mean[2]*mean[2]);
    for (i = 0; i < 3; i++)
    {
        IMU_Cal_Data.PoseAcc[IMU_Cal_Data.PoseCount][i]  = mean[i];
        IMU_Cal_Data.PoseUnit[IMU_Cal_Data.PoseCount][i] = (n > 1e-6f) ? (mean[i]/n) : 0.0f;
    }
    IMU_Cal_Data.PoseCount++;
}

/* 通用高斯-若尔当消元: 解 N(n x n) * x = rhs, n<=7; 返回 1 成功 0 奇异 */
uint8_t IMU_Cal_GaussSolve(const CAL_REAL *N, const CAL_REAL *rhs, CAL_REAL *x, int n)
{
    static CAL_REAL m[7][8];                        /* 函数内暂存矩阵, 用 static 省任务栈 */
    int i, j, k;
    if (n < 1 || n > 7) return 0;

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++) m[i][j] = N[i * n + j];
        m[i][n] = rhs[i];
    }

    for (k = 0; k < n; k++)
    {
        int piv = k;
        CAL_REAL mx = CAL_ABS(m[k][k]);
        for (i = k + 1; i < n; i++)
        {
            CAL_REAL v = CAL_ABS(m[i][k]);
            if (v > mx) { mx = v; piv = i; }
        }
        if (mx < (CAL_REAL)1e-12) return 0;         /* 奇异矩阵, 无法求解 */
        if (piv != k)                               /* 交换主元行 */
        {
            for (j = k; j <= n; j++)
            {
                CAL_REAL t = m[k][j];
                m[k][j] = m[piv][j];
                m[piv][j] = t;
            }
        }
        {                                           /* 主元行归一化 */
            CAL_REAL d = m[k][k];
            for (j = k; j <= n; j++) m[k][j] /= d;
        }
        for (i = 0; i < n; i++)                     /* 消去其它行的当前列 */
        {
            CAL_REAL f;
            if (i == k) continue;
            f = m[i][k];
            if (f == (CAL_REAL)0.0) continue;
            for (j = k; j <= n; j++) m[i][j] -= f * m[k][j];
        }
    }
    for (i = 0; i < n; i++) x[i] = m[i][n];
    return 1;
}

/*
 * 加速度计对角拟合(两步法, 数值稳定)
 * 模型:  cal_i = s_i * (p_i - b_i), 静止时要求 |cal| = g。
 * 第1步: 假设 s≈1, 用 4 参数最小二乘求球心(即零偏 b):
 *        |p - b|^2 = g^2  =>  (-2b)·p + (|b|^2 - g^2) = -|p|^2
 *        每姿态一行 [px, py, pz, 1], 目标值 -(px^2+py^2+pz^2)
 * 第2步: 固定 b, 用 3 参数最小二乘求每轴 s^2:
 *        每姿态一行 [(px-bx)^2, (py-by)^2, (pz-bz)^2], 目标值 g^2
 * 这样避免了“姿态近似落在一个球面上导致尺度与常数列近线性相关”
 * 的病态问题(7 参数线性化在球面数据上会退化)。
 * 返回值: 0 = 无法拟合, 1 = 拟合成功(残差合格),
 *         2 = 已拟合但姿态散布过大(残差不合格)。
 */
uint8_t IMU_Cal_Fit_Accel(void)
{
    static CAL_REAL NA[4][4], rA[4], xA[4];         /* 第1步 4x4 正规方程 */
    static CAL_REAL NB[3][3], rB[3], xB[3];         /* 第2步 3x3 正规方程 */
    static float    calNorm[IMU_CAL_POSE_MAX];      /* 各姿态的校准模长   */
    const CAL_REAL g2 = (CAL_REAL)IMU_CAL_G_REF * (CAL_REAL)IMU_CAL_G_REF;
    CAL_REAL bo[3], sc[3];
    uint8_t r, i;
    int a, c;

    if (IMU_Cal_Data.PoseCount < IMU_CAL_POSE_MIN) return 0;

    /* ---- 第1步: 求零偏 b(把各姿态视作同一个 g 的球面) ---- */
    for (a = 0; a < 4; a++)
    {
        rA[a] = (CAL_REAL)0.0;
        for (c = 0; c < 4; c++) NA[a][c] = (CAL_REAL)0.0;
    }
    for (r = 0; r < IMU_Cal_Data.PoseCount; r++)
    {
        CAL_REAL px = IMU_Cal_Data.PoseAcc[r][0];
        CAL_REAL py = IMU_Cal_Data.PoseAcc[r][1];
        CAL_REAL pz = IMU_Cal_Data.PoseAcc[r][2];
        CAL_REAL row[4] = { px, py, pz, (CAL_REAL)1.0 };
        CAL_REAL tgt = -(px*px + py*py + pz*pz);
        for (a = 0; a < 4; a++)
        {
            rA[a] += row[a] * tgt;
            for (c = 0; c < 4; c++) NA[a][c] += row[a] * row[c];
        }
    }
    if (!IMU_Cal_GaussSolve(&NA[0][0], rA, xA, 4)) return 0;
    for (i = 0; i < 3; i++)
    {
        bo[i] = -(CAL_REAL)0.5 * xA[i];             /* xA[0..2] = -2*b */
        if (CAL_ABS(bo[i]) > (CAL_REAL)4.0) return 0; /* 零偏明显异常 */
    }

    /* ---- 第2步: 求每轴比例 s(固定 b) ---- */
    for (a = 0; a < 3; a++)
    {
        rB[a] = (CAL_REAL)0.0;
        for (c = 0; c < 3; c++) NB[a][c] = (CAL_REAL)0.0;
    }
    for (r = 0; r < IMU_Cal_Data.PoseCount; r++)
    {
        CAL_REAL dx = IMU_Cal_Data.PoseAcc[r][0] - bo[0];
        CAL_REAL dy = IMU_Cal_Data.PoseAcc[r][1] - bo[1];
        CAL_REAL dz = IMU_Cal_Data.PoseAcc[r][2] - bo[2];
        CAL_REAL row[3] = { dx*dx, dy*dy, dz*dz };
        for (a = 0; a < 3; a++)
        {
            rB[a] += row[a] * g2;
            for (c = 0; c < 3; c++) NB[a][c] += row[a] * row[c];
        }
    }
    if (!IMU_Cal_GaussSolve(&NB[0][0], rB, xB, 3)) return 0;
    for (i = 0; i < 3; i++)
    {
        if (xB[i] <= (CAL_REAL)1e-9) return 0;      /* s^2 必须为正 */
        sc[i] = CAL_SQRT(xB[i]);
        if (sc[i] < (CAL_REAL)0.5 || sc[i] > (CAL_REAL)2.0) return 0; /* 比例异常 */
    }

    /* 一致性指标: 各姿态校准模长相对其均值的最大离散 */
    {
        CAL_REAL mean = (CAL_REAL)0.0, worst = (CAL_REAL)0.0;
        for (r = 0; r < IMU_Cal_Data.PoseCount; r++)
        {
            CAL_REAL dx = (IMU_Cal_Data.PoseAcc[r][0] - bo[0]) * sc[0];
            CAL_REAL dy = (IMU_Cal_Data.PoseAcc[r][1] - bo[1]) * sc[1];
            CAL_REAL dz = (IMU_Cal_Data.PoseAcc[r][2] - bo[2]) * sc[2];
            calNorm[r] = (float)CAL_SQRT(dx*dx + dy*dy + dz*dz);
            mean += calNorm[r];
        }
        mean /= (CAL_REAL)IMU_Cal_Data.PoseCount;
        for (r = 0; r < IMU_Cal_Data.PoseCount; r++)
        {
            CAL_REAL d = CAL_ABS((CAL_REAL)calNorm[r] - mean);
            if (d > worst) worst = d;
        }
        if (worst > (CAL_REAL)IMU_CAL_RESID_TOL) return 2;  /* 散布过大 */
    }

    /* 归一化: 使各姿态校准模长的平均值等于参考重力 g */
    {
        CAL_REAL mean = (CAL_REAL)0.0;
        for (r = 0; r < IMU_Cal_Data.PoseCount; r++) mean += calNorm[r];
        mean /= (CAL_REAL)IMU_Cal_Data.PoseCount;
        if (mean > (CAL_REAL)1e-6)
        {
            CAL_REAL k = (CAL_REAL)IMU_CAL_G_REF / mean;
            for (i = 0; i < 3; i++) sc[i] *= k;
        }
    }

    /* 保存结果到 Config */
    for (i = 0; i < 3; i++)
    {
        IMU_Cal_Data.Config.Acc_Offset[i] = (float)bo[i];
        IMU_Cal_Data.Config.Acc_Scale[i]  = (float)sc[i];
    }
    return 1;
}

/*
 * 由两个不平行的参考姿态构造输出旋转矩阵 Frame_R。
 *   u = 归一化( (RefUpRaw  - Acc_Offset) .* Acc_Scale ): 芯片坐标下“正面朝上”的竖直朝上单位向量
 *   f = 归一化( (RefFwdRaw - Acc_Offset) .* Acc_Scale ): 芯片坐标下“Pitch ~90°”的单位向量
 * 构造正交基(行向量) R = [xr; yr; zr]:
 *   zr = u                    (输出 +z = 竖直朝上)
 *   xr = normalize( f - (f·u)u )  (输出 +x = 模块前向, 由 Pitch 方向确定)
 *   yr = u × xr               (输出 +y, 右手系)
 * 两个矢量即可完整确定芯片->模块旋转(含绕竖直轴的方位), 不再依赖
 * “芯片 x 投影到水平面”的假设 -> 即使芯片相对模块有旋转/倾斜安装也正确。
 * 返回 1 成功 0 失败。
 */
uint8_t IMU_Cal_Compute_Frame(void)
{
    const float *off = IMU_Cal_Data.Config.Acc_Offset;
    const float *scl = IMU_Cal_Data.Config.Acc_Scale;
    CAL_REAL cx, cy, cz, n;
    CAL_REAL ux, uy, uz;
    CAL_REAL fx, fy, fz, nf;
    CAL_REAL dot, xrx, xry, xrz, nxr;
    CAL_REAL yrx, yry, yrz;

    /* 校正后的“正面朝上”矢量 -> u (输出 +z) */
    cx = (CAL_REAL)(IMU_Cal_Data.RefUpRaw[0] - off[0]) * (CAL_REAL)scl[0];
    cy = (CAL_REAL)(IMU_Cal_Data.RefUpRaw[1] - off[1]) * (CAL_REAL)scl[1];
    cz = (CAL_REAL)(IMU_Cal_Data.RefUpRaw[2] - off[2]) * (CAL_REAL)scl[2];
    n  = CAL_SQRT(cx*cx + cy*cy + cz*cz);
    if (n < (CAL_REAL)1e-6) return 0;
    ux = cx / n; uy = cy / n; uz = cz / n;          /* u = 输出 +z (芯片坐标) */

    /* 校正后的“Pitch ~90°”矢量 -> f */
    cx = (CAL_REAL)(IMU_Cal_Data.RefFwdRaw[0] - off[0]) * (CAL_REAL)scl[0];
    cy = (CAL_REAL)(IMU_Cal_Data.RefFwdRaw[1] - off[1]) * (CAL_REAL)scl[1];
    cz = (CAL_REAL)(IMU_Cal_Data.RefFwdRaw[2] - off[2]) * (CAL_REAL)scl[2];
    nf = CAL_SQRT(cx*cx + cy*cy + cz*cz);
    if (nf < (CAL_REAL)1e-6) return 0;
    fx = cx / nf; fy = cy / nf; fz = cz / nf;

    /* xr: 前向在 ⊥u 平面内的分量(若 f≈±u 则退化; 上层已校验 60°~120°) */
    dot = ux*fx + uy*fy + uz*fz;
    xrx = fx - dot*ux;
    xry = fy - dot*uy;
    xrz = fz - dot*uz;
    nxr = CAL_SQRT(xrx*xrx + xry*xry + xrz*xrz);
    if (nxr < (CAL_REAL)1e-4) return 0;
    xrx /= nxr; xry /= nxr; xrz /= nxr;

    /* yr = u × xr (右手系) */
    yrx = uy*xrz - uz*xry;
    yry = uz*xrx - ux*xrz;
    yrz = ux*xry - uy*xrx;

    /* 填入 Frame_R: 行 = 输出轴在芯片坐标中的表示 */
    IMU_Cal_Data.Config.Frame_R[0][0] = (float)xrx; IMU_Cal_Data.Config.Frame_R[0][1] = (float)xry; IMU_Cal_Data.Config.Frame_R[0][2] = (float)xrz;
    IMU_Cal_Data.Config.Frame_R[1][0] = (float)yrx; IMU_Cal_Data.Config.Frame_R[1][1] = (float)yry; IMU_Cal_Data.Config.Frame_R[1][2] = (float)yrz;
    IMU_Cal_Data.Config.Frame_R[2][0] = (float)ux;  IMU_Cal_Data.Config.Frame_R[2][1] = (float)uy;  IMU_Cal_Data.Config.Frame_R[2][2] = (float)uz;
    return 1;
}

/* 校准收尾: 计算陀螺零漂、置完成标志并打印校准参数 */
void IMU_Cal_Finish(void)
{
    uint8_t i;

    if (IMU_Cal_Data.GyrCnt > 0)                    /* 零漂 = 静止样本均值 */
    {
        for (i = 0; i < 3; i++)
            IMU_Cal_Data.Config.Gyro_Offset[i] = IMU_Cal_Data.GyrSum[i] / (float)IMU_Cal_Data.GyrCnt;
        IMU_Cal_Data.GyroCalValid = 1u;             /* 人机交互标定确实实测了零漂 */
    }

    IMU_Cal_Data.Config.Is_Calibrated = 1;
    IMU_Cal_Data.CalState = IMU_CAL_STATE_DONE;

    /* 校准值固定打印, 便于记录(不随调试开关关闭) */
    IMU_CAL_PRINTF("\r\n============ IMU CALIBRATION DONE ============\r\n");
    IMU_CAL_PRINTF("Acc_Offset(m/s2)  : %.6f, %.6f, %.6f\r\n",
                   IMU_Cal_Data.Config.Acc_Offset[0],
                   IMU_Cal_Data.Config.Acc_Offset[1],
                   IMU_Cal_Data.Config.Acc_Offset[2]);
    IMU_CAL_PRINTF("Acc_Scale         : %.6f, %.6f, %.6f\r\n",
                   IMU_Cal_Data.Config.Acc_Scale[0],
                   IMU_Cal_Data.Config.Acc_Scale[1],
                   IMU_Cal_Data.Config.Acc_Scale[2]);
    IMU_CAL_PRINTF("Gyro_Offset(rad/s): %.6f, %.6f, %.6f\r\n",
                   IMU_Cal_Data.Config.Gyro_Offset[0],
                   IMU_Cal_Data.Config.Gyro_Offset[1],
                   IMU_Cal_Data.Config.Gyro_Offset[2]);
    IMU_CAL_PRINTF("Frame_R (chip->out):\r\n");
    for (i = 0; i < 3; i++)
    {
        IMU_CAL_PRINTF("  %8.6f %8.6f %8.6f\r\n",
                       (double)IMU_Cal_Data.Config.Frame_R[i][0],
                       (double)IMU_Cal_Data.Config.Frame_R[i][1],
                       (double)IMU_Cal_Data.Config.Frame_R[i][2]);
    }
    IMU_CAL_PRINTF("check: place FRONT face UP & still => acc should be 0,0,+g\r\n");
    IMU_CAL_PRINTF("calibration finished. call IMU_Cal_Get_Calibrated_Data() for data.\r\n");
}

/*
 * 姿态足够时尝试拟合。返回:
 *   1 = 拟合成功 -> 应切换到阶段2(参考面锁定)
 *   0 = 继续采集姿态
 */
uint8_t IMU_Cal_Try_Fit(void)
{
    uint8_t st;
    if (IMU_Cal_Data.PoseCount < IMU_CAL_POSE_MIN) return 0;

    st = IMU_Cal_Fit_Accel();                       /* 0失败 1成功 2散布过大 */
    if (st == 1u) return 1u;                        /* 拟合成功, 去参考面 */

    if (st == 0u)                                   /* 奇异 / 无效拟合 */
    {
        if (IMU_Cal_Data.PoseCount >= IMU_CAL_POSE_MAX)
        {
            IMU_CAL_LOG("[Cal] fit failed, restarting pose collection...\r\n");
            IMU_Cal_Data.PoseCount = 0;             /* 让操作者重新采集 */
        }
        else
        {
            IMU_CAL_LOG("[Cal] poses %u/%u not spread enough, flip to noticeably\r\n",
                        (uint32_t)IMU_Cal_Data.PoseCount, (uint32_t)IMU_CAL_POSE_MAX);
            IMU_CAL_LOG("      different directions (tilt each axis) and hold still.\r\n");
        }
        return 0u;
    }

    /* st == 2: 已拟合但散布过大 */
    if (IMU_Cal_Data.PoseCount >= IMU_CAL_POSE_MAX) return 1u;   /* 已达上限, 尽力使用当前拟合 */

    IMU_CAL_LOG("[Cal] poses %u/%u not spread enough, flip to noticeably different\r\n",
                (uint32_t)IMU_Cal_Data.PoseCount, (uint32_t)IMU_CAL_POSE_MAX);
    IMU_CAL_LOG("      directions (tilt each axis) and hold still again.\r\n");
    return 0u;
}
