/**
 ******************************************************************************
 * @file    BMP581.c
 * @brief   BMP581 气压计驱动(V3 控制板 U8, 硬件 SPI1 + 数据就绪中断)
 *
 *  移植自旧 SPA06(I2C, 需要读标定系数做补偿) -> BMP581 片内已补偿:
 *      Temperature = raw_T / 65536        (degC)
 *      Pressure    = raw_P / 64           (Pa)
 *  海拔仍用国际气压测高公式, 相对高度以上电首个有效样本为零点。
 *
 *  采样方式: 50Hz 连续测量(ODR=50Hz), 每次转换完成 INT(PB1) 输出一个
 *  低电平脉冲 -> EXTI1 下降沿 -> BMP581_INT_Callback() 置标志 ->
 *  Sensor 任务里 BMP581_Read_Data() 才走一次 SPI。
 *  (旧代码是每 20ms 无条件读, 会读到重复样本; 现在由 INT 对齐新样本.)
 ******************************************************************************
 */
#include "BMP581.h"
#include "math.h"

BMP581_StructTypedef BMP581;

/* INT 标志: 在 EXTI 中断里写, 主循环里读(BMP581.Int_Count 也由中断累加) */
static volatile uint8_t BMP581_INT_Flag = 0;

/* 最近一次读数据的时刻(ms), 用于 INT 超时兜底 */
static uint32_t BMP581_LastReadMs = 0;

/* 相对高度零点 */
static float   BMP581_Height_Zero  = 0.0f;
static uint8_t BMP581_Height_Ready = 0;

/* INT 一直不来时的兜底时间 / 有效气压范围 / ID 重试次数 见 Parameter.h 第 4 节 */

/*============================ 寄存器底层(SPI1) ============================*/
uint8_t BMP581_Read_Reg(BMP581_Reg_EnumTypedef Reg)
{
    uint8_t data = 0;
    BMP581_Read_RegList(Reg, 1, &data);
    return data;
}

void BMP581_Read_RegList(BMP581_Reg_EnumTypedef Reg, uint8_t Num, uint8_t *Read_Data)
{
    uint8_t tx[8] = {0};
    uint8_t rx[8] = {0};

    if ((Read_Data == NULL) || (Num == 0)) return;
    if (Num > 7) Num = 7;

    tx[0] = (uint8_t)((uint8_t)Reg | 0x80);     /* bit7=1: 读 */

    BMP581_CS(0);
    HAL_SPI_TransmitReceive(&BMP581_SPI, tx, rx, (uint16_t)(Num + 1u), 10);
    BMP581_CS(1);

    memcpy(Read_Data, &rx[1], Num);
}

void BMP581_Write_Reg(BMP581_Reg_EnumTypedef Reg, uint8_t Write_Data)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (uint8_t)((uint8_t)Reg & 0x7F);     /* bit7=0: 写 */
    tx[1] = Write_Data;

    BMP581_CS(0);
    HAL_SPI_TransmitReceive(&BMP581_SPI, tx, rx, 2, 10);
    BMP581_CS(1);
}

/*=============================== 初始化 ==================================*/
/* 读芯片 ID: 上电/软复位后头几次可能读不到(接口切换/上电时序),
 * 一次失败就放弃会让 Ready 永久为 0 -> 表现为"传感器有时好有时死"。
 * 所以这里循环重试(次数/间隔见 Parameter.h 的 BMP581_ID_RETRY)。          */
static uint8_t BMP581_Wait_Chip_ID(void)
{
    uint8_t id = 0u;
    uint8_t i;

    for (i = 0u; i < BMP581_ID_RETRY; i++)
    {
        id = BMP581_Read_Reg(BMP581_Reg_CHIP_ID);
        if (id == BMP581_CHIP_ID_VALUE) return id;
        HAL_Delay(5);
    }
    return id;
}

void BMP581_Init(void)
{
    uint8_t id;

    BMP581_INT_Flag     = 0;
    BMP581.Int_Count    = 0;
    BMP581.Poll_Count   = 0;
    BMP581_Height_Ready = 0;
    BMP581.Ready        = 0;
    BMP581.Sample_Count = 0;

    BMP581_CS(1);
    HAL_Delay(2);

    /* 1. 读 ID: 失败时自动重试(最多 250ms) */
    BMP581.Chip_ID = id = BMP581_Wait_Chip_ID();
    if (id != BMP581_CHIP_ID_VALUE)
    {
        /* 器件不对(或 SPI 没通): 不继续配置, 保持 Ready=0, 上层可看 Chip_ID 定位 */
        return;
    }

    /* 2. 软复位, 清掉之前可能残留的状态 */
    BMP581_Write_Reg(BMP581_Reg_CMD, 0xB6);
    HAL_Delay(10);

    /* 软复位后接口要重新建立, 同样重试 */
    BMP581.Chip_ID = id = BMP581_Wait_Chip_ID();
    if (id != BMP581_CHIP_ID_VALUE)
    {
        return;
    }

    /* 3. OSR: 气压 8x, 温度 2x, 打开气压测量(温度始终开启)
     *    过采样越高噪声越低, 但转换时间变长; 50Hz ODR 下 8x/2x 余量充足 */
    BMP581_Write_Reg(BMP581_Reg_OSR_CONFIG, 0x40u | (3u << 3) | 1u);

    /* 4. ODR = 50Hz, 先留在 standby 写配置 */
    BMP581_Write_Reg(BMP581_Reg_ODR_CONFIG, (0x0Fu << 2) | 0x00u);

    /* 5. INT: 使能 + 推挽 + 低电平有效 + **脉冲模式**
     *    bit3 en | bit2 od(0=推挽) | bit1 pol(0=低有效) | bit0 mode(1=脉冲)
     *    ★ bit0 = 0 是"锁存"模式: INT 拉低后必须读 INT_STATUS 才能释放,
     *      不读就会一直低 —— 表现为后续再也没中断(Int_Count 只跳 1 次),
     *      只能靠 100ms 兜底轮询。所以这里必须是 1(脉冲, 自动释放) = 0x09。 */
    BMP581_Write_Reg(BMP581_Reg_INT_CONFIG, 0x09u);

    /* 6. INT 源: 数据就绪 */
    BMP581_Write_Reg(BMP581_Reg_INT_SOURCE, 0x01u);

    /* 7. 进入 normal 连续测量模式 */
    BMP581_Write_Reg(BMP581_Reg_ODR_CONFIG, (0x0Fu << 2) | 0x01u);

    HAL_Delay(10);      /* 等第一帧转换完成 */

    BMP581.Ready = 1;
}

/*============================= 读数据 ====================================*/
uint8_t BMP581_Read_Data(void)
{
    uint8_t  buf[6] = {0};
    uint32_t rawT, rawP;
    uint32_t now;
    uint8_t  new_data = 0;

    if (!BMP581.Ready) return 0;

    now = HAL_GetTick();

    /* 正常情况下只由 INT 触发读取; 超过 TIMEOUT 没有中断才兜底轮询一次 */
    if (BMP581_INT_Flag)
    {
        BMP581_INT_Flag = 0;
        new_data = 1;
    }
    else if ((uint32_t)(now - BMP581_LastReadMs) > BMP581_INT_TIMEOUT_MS)
    {
        BMP581.Poll_Count++;
        new_data = 1;
    }

    if (!new_data) return 0;

    BMP581_LastReadMs = now;

    /* 一次连贯读 0x1D~0x22 (温度 3 字节 + 气压 3 字节) */
    BMP581_Read_RegList(BMP581_Reg_TEMP_XLSB, 6, buf);

    /* 温度: 24bit 有符号, 1 LSB = 1/65536 degC */
    rawT = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];
    if (rawT & 0x800000u) rawT |= 0xFF000000u;

    /* 气压: 24bit 无符号, 1 LSB = 1/64 Pa */
    rawP = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[3];

    {
        float pressure = (float)rawP / 64.0f;

        /* 量程外的值(未转换完/通信错)直接丢弃, 保持上一帧 */
        if ((pressure < BMP581_PRESS_MIN) || (pressure > BMP581_PRESS_MAX)) return 0;

        BMP581.Temperature = (float)(int32_t)rawT / 65536.0f;
        BMP581.Pressure    = pressure;

        /* 国际气压测高公式 */
        {
            double ratio = pow((double)pressure / (double)BMP581_SEA_PRESSURE, 1.0 / 5.255);
            BMP581.Altitude = (float)(44330.0 * (1.0 - ratio));
        }

        /* 相对高度: 以上电首个有效海拔为零点 */
        if (!BMP581_Height_Ready)
        {
            BMP581_Height_Zero  = BMP581.Altitude;
            BMP581_Height_Ready = 1;
        }
        BMP581.Relative_Height = BMP581.Altitude - BMP581_Height_Zero;
        BMP581.Relative_Height_mm = (int32_t)(BMP581.Relative_Height * 1000.0f +
                                              (BMP581.Relative_Height >= 0.0f ? 0.5f : -0.5f));
    }

    BMP581.Sample_Count++;

    /* ★ 释放中断: INT_STATUS(0x27) 的 drdy 标志必须读一次才会清。
     *   不读的话, 若 INT 工作在锁存模式, 引脚会一直拉低 -> 后续再也没下降沿,
     *   表现为 Int_Count 只跳一次、只能靠 100ms 兜底轮询(实测就是这个现象)。 */
    (void)BMP581_Read_Reg(BMP581_Reg_INT_STATUS);

    return 1;
}

/*=========================== 相对高度零点重置 ============================*/
void BMP581_Reset_Height(void)
{
    if (!BMP581_Height_Ready) return;

    BMP581_Height_Zero        = BMP581.Altitude;
    BMP581.Relative_Height    = 0.0f;
    BMP581.Relative_Height_mm = 0;
}

/*============================= EXTI 回调 =================================*/
/* 在 HAL_GPIO_EXTI_Callback 里(BPS_INT_Pin)调用: 只置标志, 不做 SPI */
void BMP581_INT_Callback(void)
{
    BMP581_INT_Flag = 1;
    BMP581.Int_Count++;
}
