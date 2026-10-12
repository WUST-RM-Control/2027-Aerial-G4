/*============================================================================
 * VL53L1.c  --  单目标测距驱动(纯寄存器版)
 *--------------------------------------------------------------------------
 * 来源: 移植自 Code\文档\vl53l1.c / vl53l1.h (第三方精简驱动)
 * 说明: 不使用 ST 官方 vl53l1 API, 直接写寄存器配置 + 读 RESULT 寄存器。
 *       挂在 I2C3(hi2c3), 器件地址 0x29(HAL 8位 0x52); 不依赖 GPIO1 引脚,
 *       就绪判定走内部 pad 状态寄存器(0x0031)。
 * 总线: 8位寄存器索引; I2C 传输为 16位大端寄存器地址 + 数据(HAL Mem 读写)。
 * 默认: 长程 Long(0~4m) + 100ms 预算, 可在初始化后调 setDistanceModeShort() 切换。
 *==========================================================================*/

#include "VL53L1.h"
#include <string.h>

/* ===================== 寄存器定义(移植自文档 vl53l1.h) ===================== */
#define VL53L1_SOFT_RESET                                         0x0000
#define VL53L1_I2C_SLAVE__DEVICE_ADDRESS                          0x0001
#define VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND              0x0008

#define VL53L1_ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS     0x0016
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS 0x0018
#define VL53L1_ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS 0x001A
#define VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM                 0x001E
#define VL53L1_MM_CONFIG__INNER_OFFSET_MM                         0x0020
#define VL53L1_MM_CONFIG__OUTER_OFFSET_MM                         0x0022

#define VL53L1_GPIO_HV_MUX__CTRL                                  0x0030
#define VL53L1_GPIO__TIO_HV_STATUS                                0x0031

#define VL53L1_SYSTEM__INTERRUPT_CONFIG_GPIO                      0x0046
#define VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP                    0x004B

#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI                  0x005E
#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_LO                  0x005F
#define VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A                       0x0060
#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI                  0x0061
#define VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_LO                  0x0062
#define VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B                       0x0063
#define VL53L1_RANGE_CONFIG__SIGMA_THRESH                         0x0064
#define VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS        0x0066
#define VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH                     0x0069
#define VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD                    0x006C

#define VL53L1_SD_CONFIG__WOI_SD0                                 0x0078
#define VL53L1_SD_CONFIG__INITIAL_PHASE_SD0                       0x007A
#define VL53L1_ROI_CONFIG__USER_ROI_CENTRE_SPAD                   0x007F
#define VL53L1_ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE      0x0080
#define VL53L1_SYSTEM__SEQUENCE_CONFIG                            0x0081
#define VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD                     0x0082

#define VL53L1_SYSTEM__INTERRUPT_CLEAR                            0x0086
#define VL53L1_SYSTEM__MODE_START                                 0x0087

#define VL53L1_RESULT__RANGE_STATUS                               0x0089
#define VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0             0x008C
#define VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0                0x0090
#define VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0     0x0096
#define VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 0x0098

/* ===================== 全局结构体 ===================== */
VL53L1_StructTypedef VL53L1;

/* ===================== 内部函数声明 ===================== */
static uint8_t  getDistanceMode(void);              // 1=短程 2=长程
static uint16_t getTimingBudgetInMs(void);          // 读当前预算
static void     setDistanceMode(uint16_t DM);       // 1短 2长
static void     startRanging(void);
static void     stopRanging(void);
static uint8_t  checkForDataReady(void);
static void     clearInterrupt(void);
static uint8_t  getInterruptPolarity(void);
static void     writeByteData(uint16_t index, uint8_t data);
static void     writeWordData(uint16_t index, uint16_t data);
static void     readByteData(uint16_t index, uint8_t *data);
static void     readWordData(uint16_t index, uint16_t *data);

/* ===================== I2C 底层(HAL I2C3, 16位寄存器地址) ===================== */
static void I2C3_WriteReg(uint16_t reg, uint8_t *pBuf, uint16_t len)
{
    HAL_I2C_Mem_Write(&hi2c3, VL53L1_I2C_ADDR_BYTE, reg,
                      I2C_MEMADD_SIZE_16BIT, pBuf, len, 1000);
}

static void I2C3_ReadReg(uint16_t reg, uint8_t *pBuf, uint16_t len)
{
    HAL_I2C_Mem_Read(&hi2c3, VL53L1_I2C_ADDR_BYTE, reg,
                     I2C_MEMADD_SIZE_16BIT, pBuf, len, 1000);
}

/* ===================== 默认配置表(移植自原驱动, 0x2D~0x87 逐字节写) ===================== */
static const uint8_t VL53L1_DEFAULT_CONFIGURATION[] = {
0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
0x00, /* 0x32 : not user-modifiable */
0x02, /* 0x33 : not user-modifiable */
0x08, /* 0x34 : not user-modifiable */
0x00, /* 0x35 : not user-modifiable */
0x08, /* 0x36 : not user-modifiable */
0x10, /* 0x37 : not user-modifiable */
0x01, /* 0x38 : not user-modifiable */
0x01, /* 0x39 : not user-modifiable */
0x00, /* 0x3a : not user-modifiable */
0x00, /* 0x3b : not user-modifiable */
0x00, /* 0x3c : not user-modifiable */
0x00, /* 0x3d : not user-modifiable */
0xff, /* 0x3e : not user-modifiable */
0x00, /* 0x3f : not user-modifiable */
0x0F, /* 0x40 : not user-modifiable */
0x00, /* 0x41 : not user-modifiable */
0x00, /* 0x42 : not user-modifiable */
0x00, /* 0x43 : not user-modifiable */
0x00, /* 0x44 : not user-modifiable */
0x00, /* 0x45 : not user-modifiable */
0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
0x0b, /* 0x47 : not user-modifiable */
0x00, /* 0x48 : not user-modifiable */
0x00, /* 0x49 : not user-modifiable */
0x02, /* 0x4a : not user-modifiable */
0x0a, /* 0x4b : not user-modifiable */
0x21, /* 0x4c : not user-modifiable */
0x00, /* 0x4d : not user-modifiable */
0x00, /* 0x4e : not user-modifiable */
0x05, /* 0x4f : not user-modifiable */
0x00, /* 0x50 : not user-modifiable */
0x00, /* 0x51 : not user-modifiable */
0x00, /* 0x52 : not user-modifiable */
0x00, /* 0x53 : not user-modifiable */
0xc8, /* 0x54 : not user-modifiable */
0x00, /* 0x55 : not user-modifiable */
0x00, /* 0x56 : not user-modifiable */
0x38, /* 0x57 : not user-modifiable */
0xff, /* 0x58 : not user-modifiable */
0x01, /* 0x59 : not user-modifiable */
0x00, /* 0x5a : not user-modifiable */
0x08, /* 0x5b : not user-modifiable */
0x00, /* 0x5c : not user-modifiable */
0x00, /* 0x5d : not user-modifiable */
0x01, /* 0x5e : not user-modifiable */
0xdb, /* 0x5f : not user-modifiable */
0x0f, /* 0x60 : not user-modifiable */
0x01, /* 0x61 : not user-modifiable */
0xf1, /* 0x62 : not user-modifiable */
0x0d, /* 0x63 : not user-modifiable */
0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
0x68, /* 0x65 : Sigma threshold LSB */
0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
0x80, /* 0x67 : Min count Rate LSB */
0x08, /* 0x68 : not user-modifiable */
0xb8, /* 0x69 : not user-modifiable */
0x00, /* 0x6a : not user-modifiable */
0x00, /* 0x6b : not user-modifiable */
0x00, /* 0x6c : interMeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
0x00, /* 0x6d : interMeasurement period */
0x0f, /* 0x6e : interMeasurement period */
0x89, /* 0x6f : interMeasurement period LSB */
0x00, /* 0x70 : not user-modifiable */
0x00, /* 0x71 : not user-modifiable */
0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x73 : distance threshold high LSB */
0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
0x00, /* 0x75 : distance threshold low LSB */
0x00, /* 0x76 : not user-modifiable */
0x01, /* 0x77 : not user-modifiable */
0x0f, /* 0x78 : not user-modifiable */
0x0d, /* 0x79 : not user-modifiable */
0x0e, /* 0x7a : not user-modifiable */
0x0e, /* 0x7b : not user-modifiable */
0x00, /* 0x7c : not user-modifiable */
0x00, /* 0x7d : not user-modifiable */
0x02, /* 0x7e : not user-modifiable */
0xc7, /* 0x7f : ROI center, use SetROI() */
0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
0x9B, /* 0x81 : not user-modifiable */
0x00, /* 0x82 : not user-modifiable */
0x00, /* 0x83 : not user-modifiable */
0x00, /* 0x84 : not user-modifiable */
0x01, /* 0x85 : not user-modifiable */
0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};

/* ===================== 模式切换 ===================== */
void setDistanceModeShort(void)
{
    setDistanceMode(1);
}

void setDistanceModeLong(void)
{
    setDistanceMode(2);
}

static void setDistanceMode(uint16_t DM)
{
    uint16_t TB;

    TB = getTimingBudgetInMs();
    switch (DM) {
    case 1:   /* 短程 Short 0~1.3m */
        writeByteData(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
        writeByteData(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A,    0x07);
        writeByteData(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B,    0x05);
        writeByteData(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH,  0x38);
        writeWordData(VL53L1_SD_CONFIG__WOI_SD0,              0x0705);
        writeWordData(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0,    0x0606);
        break;
    case 2:   /* 长程 Long 0~4m */
        writeByteData(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
        writeByteData(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A,    0x0F);
        writeByteData(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B,    0x0D);
        writeByteData(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH,  0xB8);
        writeWordData(VL53L1_SD_CONFIG__WOI_SD0,              0x0F0D);
        writeWordData(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0,    0x0E0E);
        break;
    default:
        break;
    }
    setTimingBudgetInMs(TB);
}

/* ===================== 测量预算 ===================== */
void setTimingBudgetInMs(uint16_t timingBudget)
{
    uint16_t DM;

    DM = getDistanceMode();
    if (DM == 0)
        return;
    else if (DM == 1) {   /* Short */
        switch (timingBudget) {
        case 15:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
            break;
        case 20:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            break;
        case 33:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            break;
        case 50:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
            break;
        case 100:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
            break;
        case 200:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
            break;
        case 500:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1);
            break;
        default:
            break;
        }
    } else {   /* Long */
        switch (timingBudget) {
        case 20:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
            break;
        case 33:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
            break;
        case 50:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
            break;
        case 100:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
            break;
        case 200:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
            break;
        case 500:
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
            writeWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
            break;
        default:
            break;
        }
    }
}

static uint16_t getTimingBudgetInMs(void)
{
    uint16_t Temp;
    uint16_t pTimingBudget;

    readWordData(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);
    switch (Temp) {
    case 0x0051:
    case 0x001E:
        pTimingBudget = 20;
        break;
    case 0x00D6:
    case 0x0060:
        pTimingBudget = 33;
        break;
    case 0x1AE:
    case 0x00AD:
        pTimingBudget = 50;
        break;
    case 0x02E1:
    case 0x01CC:
        pTimingBudget = 100;
        break;
    case 0x03E1:
    case 0x02D9:
        pTimingBudget = 200;
        break;
    case 0x0591:
    case 0x048F:
        pTimingBudget = 500;
        break;
    default:
        pTimingBudget = 20;
        break;
    }
    return pTimingBudget;
}

static uint8_t getDistanceMode(void)
{
    uint8_t TempDM;
    uint8_t DM;

    DM = 0;
    readByteData(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);
    if (TempDM == 0x14)
        DM = 1;
    if (TempDM == 0x0A)
        DM = 2;
    return DM;
}

/* ===================== 测距启停/就绪 ===================== */
static void startRanging(void)
{
    writeByteData(VL53L1_SYSTEM__MODE_START, 0x40);
}

static void stopRanging(void)
{
    writeByteData(VL53L1_SYSTEM__MODE_START, 0x00);
}

static void clearInterrupt(void)
{
    writeByteData(VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01);
}

static uint8_t getInterruptPolarity(void)
{
    uint8_t Temp;
    uint8_t pInterruptPolarity;

    readByteData(VL53L1_GPIO_HV_MUX__CTRL, &Temp);
    Temp = Temp & 0x10;
    pInterruptPolarity = (uint8_t)(!(Temp >> 4));
    return pInterruptPolarity;
}

static uint8_t checkForDataReady(void)
{
    uint8_t Temp;
    uint8_t IntPol;

    IntPol = getInterruptPolarity();
    readByteData(VL53L1_GPIO__TIO_HV_STATUS, &Temp);
    HAL_Delay(1);

    if ((Temp & 1) == IntPol)
        return 1;
    else
        return 0;
}

/* ===================== 偏移校正(可选) ===================== */
void setOffset(int16_t OffsetValue)
{
    int16_t Temp;

    Temp = (OffsetValue * 4);
    writeWordData(VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)Temp);
    writeWordData(VL53L1_MM_CONFIG__INNER_OFFSET_MM, 0x0);
    writeWordData(VL53L1_MM_CONFIG__OUTER_OFFSET_MM, 0x0);
}

int16_t getOffset(void)
{
    int16_t offset;
    uint16_t Temp;

    readWordData(VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM, &Temp);
    if (Temp & 0x1000) {
        Temp |= 0xE000;
    }
    offset = (int16_t)Temp;
    return offset / 4;
}

/* ===================== 寄存器读写 ===================== */
static void writeByteData(uint16_t index, uint8_t data)
{
    I2C3_WriteReg(index, &data, 1);
}

static void writeWordData(uint16_t index, uint16_t data)
{
    uint8_t buffer[2];

    buffer[0] = data >> 8;
    buffer[1] = data & 0x00FF;
    I2C3_WriteReg(index, buffer, 2);
}

static void readByteData(uint16_t index, uint8_t *data)
{
    I2C3_ReadReg(index, data, 1);
}

static void readWordData(uint16_t index, uint16_t *data)
{
    uint8_t buffer[2] = {0, 0};

    I2C3_ReadReg(index, buffer, 2);
    *data = (uint16_t)((buffer[0] << 8) + buffer[1]);
}

/* ===================== 读距离(照搬原驱动) ===================== */
uint16_t getDistance(void)
{
    uint16_t tmp;

    readWordData(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp);
    clearInterrupt();
    if (tmp > 4000) {
        tmp = 65535;
    }
    return tmp;
}

uint16_t vl53l1_readdistance(void)
{
    uint16_t distance = 0;

    startRanging();
    distance = getDistance();   /* 原驱动: start 后立即读 0x0096 */
    stopRanging();

    return distance;
}

/* ===================== 初始化 ===================== */
uint8_t vl53l1_init(void)
{
    uint16_t Addr;
    uint8_t  tmp = 0;
    uint16_t wait = 0;

    /* 写整份默认配置(0x2D ~ 0x87, 逐字节) */
    for (Addr = 0x2D; Addr <= 0x87; Addr++) {
        writeByteData(Addr, VL53L1_DEFAULT_CONFIGURATION[Addr - 0x2D]);
    }

    /* 启动一次测距并等待首帧就绪(带超时, 避免器件异常时死等) */
    startRanging();
    while (tmp == 0) {
        tmp = checkForDataReady();
        if (++wait >= 30) {   /* ~3s 超时后放弃等待, 继续 */
            break;
        }
        HAL_Delay(100);
    }
    tmp = 0;
    clearInterrupt();
    stopRanging();

    /* 双边界 VHV */
    writeByteData(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
    writeByteData(0x0B, 0);

    /* 默认: 长程 Long + 100ms 预算(10Hz) */
    setDistanceMode(2);
    setTimingBudgetInMs(100);

    return 1;
}

/* ===================== QMI 风格封装(供 Sensor 任务调用) ===================== */
void VL53L1_Init(void)
{
    /* I2C3 已由 main.c 的 MX_I2C3_Init() 初始化(i2c.c) */
    vl53l1_init();
}

uint8_t VL53L1_Read_Data(void)
{
    uint16_t d;

    d = vl53l1_readdistance();

    if (d < 4000) {
        VL53L1.Distance_mm = (float)d;
        VL53L1.Updated = 1;
        VL53L1.Status = 0;
    } else {
        VL53L1.Distance_mm = 0;
        VL53L1.Updated = 0;
        VL53L1.Status = 1;
    }

    return VL53L1.Updated;
}
