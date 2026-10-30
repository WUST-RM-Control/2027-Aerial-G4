/**
 ******************************************************************************
 * @file    BMI088.c
 * @brief   BMI088 六轴 IMU 驱动(V3 控制板 U34, 硬件 SPI2 + 双数据就绪中断)
 *
 *  寄存器/时序参考 Bosch BMI088 数据手册与 RM 官方驱动:
 *    - 加速度计: 复位(PWR_CTRL/PWR_CONF) -> ACC_CONF/ACC_RANGE
 *                -> INT1 推挽低有效输出 + INT1 映射 DRDY
 *    - 陀螺仪  : 复位 -> RANGE/BANDWIDTH/LPM1 -> INT_CTRL 使能 DRDY
 *                -> INT3 推挽低有效 + INT3 映射 DRDY
 *
 *  SPI 读时序差异(容易踩坑):
 *    加速度计: [addr|0x80] [dummy] [data0..dataN]
 *    陀螺仪  : [addr|0x80] [data0..dataN]
 *
 *  采样: 陀螺 ODR 2000Hz / 加速度 ODR 1600Hz(两者都已是 BMI088 硬件上限,
 *  寄存器值见 Parameter.h 的 BMI088_ACC_CONF_REG / BMI088_GYRO_BW_REG)。
 *  数据就绪由 INT 引脚 (PA10 / PA9) 给下降沿, 主循环只在对应标志置位时
 *  读一次, 避免像旧 QMI8658A 那样每拍都读、读到重复样本。
 *
 *  ★ 样本时间戳: EXTI 里顺手记下 DWT->CYCCNT, 读 SPI 后算出相邻两个
 *    样本的真实间隔(Gyro_Dt_Cyc)。姿态解算用它当 dt, 积分总时长永远
 *    等于真实经过了多久 —— 消除“传感器时钟 ≠ 节拍时钟”带来的抖动误差。
 ******************************************************************************
 */
#include "BMI088.h"
#include "DWT.h"       /* 样本时间戳: DWT->CYCCNT / DWT_Cyc_To_Us */

BMI088_StructTypedef BMI088;

/* INT 标志: EXTI 里置位, 主循环里清(计数字段在结构体里) */
static volatile uint8_t BMI088_Acc_INT_Flag  = 0;
static volatile uint8_t BMI088_Gyro_INT_Flag = 0;

#define BMI088_DEG2RAD          0.017453292519943295f
#define BMI088_TEMP_FACTOR      0.125f
#define BMI088_TEMP_OFFSET      23.0f
#define BMI088_GRAVITY          9.8f

/*----------------------------- 寄存器地址 ---------------------------------*/
/* 加速度计 */
#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_CHIP_ID_VAL  0x1E
#define BMI088_ACC_DATA         0x12
#define BMI088_ACC_TEMP_M       0x22
#define BMI088_ACC_TEMP_L       0x23
#define BMI088_ACC_CONF         0x40
#define BMI088_ACC_RANGE        0x41
#define BMI088_ACC_INT1_IO      0x53
#define BMI088_ACC_INT_MAP      0x58
#define BMI088_ACC_PWR_CONF     0x7C
#define BMI088_ACC_PWR_CTRL     0x7D
#define BMI088_ACC_SOFTRESET    0x7E
/* 陀螺仪 */
#define BMI088_GYRO_CHIP_ID     0x00
#define BMI088_GYRO_CHIP_ID_VAL 0x0F
#define BMI088_GYRO_DATA        0x02
#define BMI088_GYRO_RANGE       0x0F
#define BMI088_GYRO_BANDWIDTH   0x10
#define BMI088_GYRO_LPM1        0x11
#define BMI088_GYRO_SOFTRESET   0x14
#define BMI088_GYRO_INT_CTRL    0x15
#define BMI088_GYRO_INT3_IO_CONF 0x16
#define BMI088_GYRO_INT3_IO_MAP 0x18

/*============================ 寄存器底层(SPI2) ============================*/
static uint8_t BMI088_Acc_Read(uint8_t Reg)
{
    uint8_t tx[3] = {(uint8_t)(Reg | 0x80), 0x00, 0x00};
    uint8_t rx[3] = {0};

    BMI088_CS_ACC(0);
    HAL_SPI_TransmitReceive(&BMI088_SPI, tx, rx, 3, 10);
    BMI088_CS_ACC(1);
    return rx[2];
}

static void BMI088_Acc_ReadList(uint8_t Reg, uint8_t Num, uint8_t *Data)
{
    uint8_t tx[10] = {0};
    uint8_t rx[10] = {0};

    if ((Data == NULL) || (Num == 0) || (Num > 8)) return;

    tx[0] = (uint8_t)(Reg | 0x80);
    /* tx[1] 是 dummy, 数据从 rx[2] 开始 */

    BMI088_CS_ACC(0);
    HAL_SPI_TransmitReceive(&BMI088_SPI, tx, rx, (uint16_t)(Num + 2u), 10);
    BMI088_CS_ACC(1);
    memcpy(Data, &rx[2], Num);
}

static void BMI088_Acc_Write(uint8_t Reg, uint8_t Data)
{
    uint8_t tx[2] = {(uint8_t)(Reg & 0x7F), Data};
    uint8_t rx[2] = {0};

    BMI088_CS_ACC(0);
    HAL_SPI_TransmitReceive(&BMI088_SPI, tx, rx, 2, 10);
    BMI088_CS_ACC(1);
}

static uint8_t BMI088_Gyro_Read(uint8_t Reg)
{
    uint8_t tx[2] = {(uint8_t)(Reg | 0x80), 0x00};
    uint8_t rx[2] = {0};

    BMI088_CS_GYRO(0);
    HAL_SPI_TransmitReceive(&BMI088_SPI, tx, rx, 2, 10);
    BMI088_CS_GYRO(1);
    return rx[1];
}

static void BMI088_Gyro_ReadList(uint8_t Reg, uint8_t Num, uint8_t *Data)
{
    uint8_t tx[10] = {0};
    uint8_t rx[10] = {0};

    if ((Data == NULL) || (Num == 0) || (Num > 8)) return;

    tx[0] = (uint8_t)(Reg | 0x80);

    BMI088_CS_GYRO(0);
    HAL_SPI_TransmitReceive(&BMI088_SPI, tx, rx, (uint16_t)(Num + 1u), 10);
    BMI088_CS_GYRO(1);
    memcpy(Data, &rx[1], Num);
}

static void BMI088_Gyro_Write(uint8_t Reg, uint8_t Data)
{
    uint8_t tx[2] = {(uint8_t)(Reg & 0x7F), Data};
    uint8_t rx[2] = {0};

    BMI088_CS_GYRO(0);
    HAL_SPI_TransmitReceive(&BMI088_SPI, tx, rx, 2, 10);
    BMI088_CS_GYRO(1);
}

/*=============================== 初始化 ==================================*/
/* 读芯片 ID: 上电/软复位后头几次可能读不到(接口切换/上电时序),
 * 一次失败就 return 会让 Ready 永久为 0 -> 表现为"IMU 有时好有时死"。
 * 循环重试次数 BMI088_ID_RETRY 见 Parameter.h 第 4 节。 */

static uint8_t BMI088_Acc_Wait_ID(void)
{
    uint8_t id = 0u;
    uint8_t i;

    for (i = 0u; i < BMI088_ID_RETRY; i++)
    {
        id = BMI088_Acc_Read(BMI088_ACC_CHIP_ID);
        if (id == BMI088_ACC_CHIP_ID_VAL) return id;
        HAL_Delay(5);
    }
    return id;
}

static uint8_t BMI088_Gyro_Wait_ID(void)
{
    uint8_t id = 0u;
    uint8_t i;

    for (i = 0u; i < BMI088_ID_RETRY; i++)
    {
        id = BMI088_Gyro_Read(BMI088_GYRO_CHIP_ID);
        if (id == BMI088_GYRO_CHIP_ID_VAL) return id;
        HAL_Delay(5);
    }
    return id;
}

static void BMI088_Acc_Init(void)
{
    uint8_t id;

    id = BMI088_Acc_Wait_ID();
    BMI088.Chip_ID_Acc = id;
    if (id != BMI088_ACC_CHIP_ID_VAL) return;

    /* 软复位后必须等 >= 50ms, 再把加速度计从 suspend 唤醒 */
    BMI088_Acc_Write(BMI088_ACC_SOFTRESET, 0xB6);
    HAL_Delay(60);

    (void)BMI088_Acc_Read(BMI088_ACC_CHIP_ID);

    BMI088_Acc_Write(BMI088_ACC_PWR_CTRL, 0x04);     /* acc_enable            */
    HAL_Delay(5);
    BMI088_Acc_Write(BMI088_ACC_PWR_CONF, 0x00);     /* active(非 suspend)    */
    HAL_Delay(5);

    /* 0x80 必置 | 加速度带宽 normal(0x20) | ODR 1600Hz(0x0C), 见 Parameter.h */
    BMI088_Acc_Write(BMI088_ACC_CONF, BMI088_ACC_CONF_REG);
    BMI088_Acc_Write(BMI088_ACC_RANGE, BMI088_ACC_RANGE_REG);   /* 量程, 见 Parameter.h */

    /* INT1: 输出使能 + 推挽 + 低电平有效 = 0x08 */
    BMI088_Acc_Write(BMI088_ACC_INT1_IO, 0x08);
    /* INT1 映射数据就绪 */
    BMI088_Acc_Write(BMI088_ACC_INT_MAP, 0x04);
}

static void BMI088_Gyro_Init(void)
{
    uint8_t id;

    id = BMI088_Gyro_Wait_ID();
    BMI088.Chip_ID_Gyro = id;
    if (id != BMI088_GYRO_CHIP_ID_VAL) return;

    BMI088_Gyro_Write(BMI088_GYRO_SOFTRESET, 0xB6);
    HAL_Delay(40);

    (void)BMI088_Gyro_Read(BMI088_GYRO_CHIP_ID);

    BMI088_Gyro_Write(BMI088_GYRO_RANGE,     BMI088_GYRO_RANGE_REG); /* 量程, 见 Parameter.h */
    BMI088_Gyro_Write(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_BW_REG);    /* ODR 2000Hz/230Hz, 见 Parameter.h */
    BMI088_Gyro_Write(BMI088_GYRO_LPM1,      0x00);   /* normal                */
    BMI088_Gyro_Write(BMI088_GYRO_INT_CTRL,  0x80);   /* 使能 DRDY 中断        */

    /* INT3: 推挽 + 低电平有效(该寄存器 bit0=lvl, bit1=od) */
    BMI088_Gyro_Write(BMI088_GYRO_INT3_IO_CONF, 0x00);
    /* INT3 映射数据就绪 */
    BMI088_Gyro_Write(BMI088_GYRO_INT3_IO_MAP, 0x01);
}

void BMI088_Init(void)
{
    BMI088.Chip_ID_Acc   = 0u;
    BMI088.Chip_ID_Gyro  = 0u;
    BMI088.Ready         = 0;

    BMI088.Acc_Int_Count   = 0;
    BMI088.Gyro_Int_Count  = 0;
    BMI088.Acc_Poll_Count  = 0;
    BMI088.Gyro_Poll_Count = 0;
    BMI088.Temperature     = 0.0f;
    BMI088.Temp_Tick       = 0u;

    /* 样本时间戳/序号: 全部归零 -> 第一个样本不产生 dt(分母/间隔未知) */
    BMI088.Acc_Ts_Cnt      = 0u;
    BMI088.Gyro_Ts_Cnt     = 0u;
    BMI088.Acc_Sample_Cyc  = 0u;
    BMI088.Gyro_Sample_Cyc = 0u;
    BMI088.Acc_Dt_Cyc      = 0u;
    BMI088.Gyro_Dt_Cyc     = 0u;
    BMI088.Acc_Dt_Us       = 0u;
    BMI088.Gyro_Dt_Us      = 0u;
    BMI088.Acc_Seq         = 0u;
    BMI088.Gyro_Seq        = 0u;

    memset(BMI088.Accel, 0, sizeof(BMI088.Accel));
    memset(BMI088.Gyro,  0, sizeof(BMI088.Gyro));

    BMI088_CS_ACC(1);
    BMI088_CS_GYRO(1);
    HAL_Delay(5);

    BMI088_Acc_Init();
    BMI088_Gyro_Init();

    BMI088.Ready = ((BMI088.Chip_ID_Acc  == BMI088_ACC_CHIP_ID_VAL) &&
                    (BMI088.Chip_ID_Gyro == BMI088_GYRO_CHIP_ID_VAL)) ? 1u : 0u;
}

/*============================= 读数据 ====================================*/
/* 样本时刻记录: ts = 本样本“数据就绪”那一刻的 DWT 周期数。
 * 第 1 个样本只记时刻不算间隔; 之后每个样本都得到与前一个样本的真实间隔。 */
static void BMI088_Acc_Ts_Update(uint32_t ts)
{
    if (BMI088.Acc_Seq != 0u)
    {
        BMI088.Acc_Dt_Cyc = (uint32_t)(ts - BMI088.Acc_Sample_Cyc);
        BMI088.Acc_Dt_Us  = DWT_Cyc_To_Us(BMI088.Acc_Dt_Cyc);
    }
    BMI088.Acc_Sample_Cyc = ts;
    BMI088.Acc_Seq++;
}

static void BMI088_Gyro_Ts_Update(uint32_t ts)
{
    if (BMI088.Gyro_Seq != 0u)
    {
        BMI088.Gyro_Dt_Cyc = (uint32_t)(ts - BMI088.Gyro_Sample_Cyc);
        BMI088.Gyro_Dt_Us  = DWT_Cyc_To_Us(BMI088.Gyro_Dt_Cyc);
    }
    BMI088.Gyro_Sample_Cyc = ts;
    BMI088.Gyro_Seq++;
}

void BMI088_Read_Data(void)
{
    static uint32_t Acc_LastMs  = 0;
    static uint32_t Gyro_LastMs = 0;
    uint32_t now = HAL_GetTick();
    uint8_t  buf[8];
    int16_t  raw;

    if (!BMI088.Ready) return;

    /*---------------- 加速度计: 每来一个新样本(INT1)读一次 ----------------*/
    if (BMI088_Acc_INT_Flag || ((uint32_t)(now - Acc_LastMs) > BMI088_INT_TIMEOUT_MS))
    {
        uint32_t ts;

        if (BMI088_Acc_INT_Flag)
        {
            ts = BMI088.Acc_Ts_Cnt;          /* INT1 触发时刻(EXTI 里记的) */
        }
        else
        {
            BMI088.Acc_Poll_Count++;         /* INT 没来: 兜底轮询 */
            ts = DWT->CYCCNT;                /* 兜底时用当前时刻 */
        }
        BMI088_Acc_INT_Flag = 0;
        Acc_LastMs = now;

        BMI088_Acc_ReadList(BMI088_ACC_DATA, 6, buf);

        raw = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
        BMI088.Accel[0] = (float)raw * (BMI088_ACCEL_RANGE_G * BMI088_GRAVITY / 32768.0f);
        raw = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
        BMI088.Accel[1] = (float)raw * (BMI088_ACCEL_RANGE_G * BMI088_GRAVITY / 32768.0f);
        raw = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
        BMI088.Accel[2] = (float)raw * (BMI088_ACCEL_RANGE_G * BMI088_GRAVITY / 32768.0f);

        /* 温度: 0x22/0x23, 11bit 有符号, 0.125 degC/LSB, 偏置 23 degC */
        BMI088_Acc_ReadList(BMI088_ACC_TEMP_M, 2, buf);
        raw = (int16_t)(((uint16_t)buf[0] << 3) | (buf[1] >> 5));
        if (raw > 1023) raw -= 2048;
        BMI088.Temperature = (float)raw * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        BMI088.Temp_Tick   = now;      /* 供加热控温判断温度是否新鲜 */

        BMI088_Acc_Ts_Update(ts);      /* 样本时间戳/间隔/序号 */
    }

    /*------------------- 陀螺仪: 每来一个新样本(INT3)读一次 ---------------*/
    if (BMI088_Gyro_INT_Flag || ((uint32_t)(now - Gyro_LastMs) > BMI088_INT_TIMEOUT_MS))
    {
        uint32_t ts;

        if (BMI088_Gyro_INT_Flag)
        {
            ts = BMI088.Gyro_Ts_Cnt;         /* INT3 触发时刻(EXTI 里记的) */
        }
        else
        {
            BMI088.Gyro_Poll_Count++;
            ts = DWT->CYCCNT;
        }
        BMI088_Gyro_INT_Flag = 0;
        Gyro_LastMs = now;

        BMI088_Gyro_ReadList(BMI088_GYRO_DATA, 6, buf);

        raw = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
        BMI088.Gyro[0] = (float)raw * (BMI088_GYRO_RANGE_DPS * BMI088_DEG2RAD / 32768.0f);
        raw = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
        BMI088.Gyro[1] = (float)raw * (BMI088_GYRO_RANGE_DPS * BMI088_DEG2RAD / 32768.0f);
        raw = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
        BMI088.Gyro[2] = (float)raw * (BMI088_GYRO_RANGE_DPS * BMI088_DEG2RAD / 32768.0f);

        BMI088_Gyro_Ts_Update(ts);     /* 样本时间戳/间隔/序号 */
    }
}

/*============================= EXTI 回调 =================================*/
void BMI088_INT_Acc_Callback(void)
{
    BMI088.Acc_Ts_Cnt = DWT->CYCCNT;    /* 样本就绪瞬间打时间戳(EXTI 里只读一个寄存器) */
    BMI088_Acc_INT_Flag = 1;
    BMI088.Acc_Int_Count++;
}

void BMI088_INT_Gyro_Callback(void)
{
    BMI088.Gyro_Ts_Cnt = DWT->CYCCNT;
    BMI088_Gyro_INT_Flag = 1;
    BMI088.Gyro_Int_Count++;
}

