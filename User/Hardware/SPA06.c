/**
 ******************************************************************************
 * @file    SPA06.c
 * @brief   SPA06-003(SLP07-003) 气压计驱动 —— 官方参考驱动移植
 * @note    移植自 Infineon 官方参考 SPL07_003 驱动(文档/SPA06-003):
 *            - 通信底层由软件I2C改为硬件 I2C3(HAL)
 *            - 保留官方: 逐字节读标定系数 / rateset 过采样&采样率 /
 *              >8x 过采样时的 P_SHIFT 处理 / 官方温度&气压补偿公式
 *            - 输出补偿后的原始数据与未滤波海拔, 并基于海拔提供相对高度
 *              (相对零点), 滤波交由外部算法完成
 ******************************************************************************
 */
#include "SPA06.h"
#include "math.h"

//SPA06-003 数据
SPA06_StructTypedef SPA06;

/* ============================= 通信配置 ============================= */
#define SPA06_I2C              hi2c3
#define SPA06_I2C_ADDR_BYTE    (uint16_t)((SPA06_I2C_ADDR << 1) & 0xFF)  /* 7位地址转HAL用8位 */

/* ===================== 官方参考驱动 常量/类型 ======================= */
#define SPL07_PRESSURE_SENSOR     0
#define SPL07_TEMPERATURE_SENSOR  1

/* 标定系数(官方 spl07_003_calib_param_t) */
typedef struct
{
    int16_t c0;
    int16_t c1;
    int32_t c00;
    int32_t c10;
    int16_t c01;
    int16_t c11;
    int16_t c20;
    int16_t c21;
    int16_t c30;
    int16_t c31;
    int16_t c40;
} SPL07_Calib_Type;

/* 运行参数(官方 spl07_003_t) */
typedef struct
{
    SPL07_Calib_Type calib;      /* 标定系数 */
    uint8_t  chip_id;            /* 芯片ID(0x11) */
    int32_t  i32rawPressure;     /* 原始24bit气压 */
    int32_t  i32rawTemperature;  /* 原始24bit温度 */
    int32_t  i32kP;              /* 气压过采样比例系数 */
    int32_t  i32kT;              /* 温度过采样比例系数 */
} SPL07_Type;

static SPL07_Type spl07;

/* ======================= 驱动内部状态(判新样本) ====================== */
static int32_t  SPA06_LastRawP = -1;           // 上一拍原始气压(判新样本)
static int32_t  SPA06_LastRawT = -1;           // 上一拍原始温度(判新样本)

/* ===================== 相对高度状态(未滤波) ========================= */
static float    SPA06_Height_Zero  = 0.0f;     // 相对高度零点(绝对海拔m)
static uint8_t  SPA06_Height_Ready = 0;        // 已建立相对零点

/* ====================== 寄存器底层(硬件I2C3) ========================= */
//读单个寄存器
uint8_t SPA06_Read_Reg(uint8_t Reg)
{
    uint8_t data = 0;
    HAL_I2C_Mem_Read(&SPA06_I2C, SPA06_I2C_ADDR_BYTE, (uint16_t)Reg,
                     I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
    return data;
}

//读多个寄存器
void SPA06_Read_RegList(uint8_t Reg, uint8_t Num, uint8_t *Read_Data)
{
    HAL_I2C_Mem_Read(&SPA06_I2C, SPA06_I2C_ADDR_BYTE, (uint16_t)Reg,
                     I2C_MEMADD_SIZE_8BIT, Read_Data, Num, 1000);
}

//写单个寄存器
void SPA06_Write_Reg(uint8_t Reg, uint8_t Write_Data)
{
    HAL_I2C_Mem_Write(&SPA06_I2C, SPA06_I2C_ADDR_BYTE, (uint16_t)Reg,
                      I2C_MEMADD_SIZE_8BIT, &Write_Data, 1, 1000);
}

/* ==================== 官方参考驱动 核心算法移植 ===================== */
//读芯片ID
static void SPL07_Read_Chip_Id(void)
{
    spl07.chip_id = SPA06_Read_Reg(SPA06_Reg_ID);
}

//芯片是否为 SPL07-003(0x11)
static uint8_t SPL07_Is_003(void)
{
    return (spl07.chip_id == 0x11) ? 1 : 0;
}

//读取标定系数(官方逐字节读取, 含c31/c40的0x11分支)
static void SPL07_Get_Calib_Param(void)
{
    uint8_t h, m, l;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x00);           /* 0x10 */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x01);           /* 0x11 */
    spl07.calib.c0 = (int16_t)h << 4 | l >> 4;
    spl07.calib.c0 = (spl07.calib.c0 & 0x0800) ? (int16_t)(0xF000 | spl07.calib.c0) : spl07.calib.c0;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x01);           /* 0x11 */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x02);           /* 0x12 */
    spl07.calib.c1 = (int16_t)((h & 0x0F) << 8) | l;
    spl07.calib.c1 = (spl07.calib.c1 & 0x0800) ? (int16_t)(0xF000 | spl07.calib.c1) : spl07.calib.c1;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x03);           /* 0x13 */
    m = SPA06_Read_Reg(SPA06_Reg_COEF + 0x04);           /* 0x14 */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x05);           /* 0x15 */
    spl07.calib.c00 = (int32_t)h << 12 | (int32_t)m << 4 | (int32_t)l >> 4;
    spl07.calib.c00 = (spl07.calib.c00 & 0x080000) ? (int32_t)(0xFFF00000 | spl07.calib.c00) : spl07.calib.c00;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x05);           /* 0x15 */
    m = SPA06_Read_Reg(SPA06_Reg_COEF + 0x06);           /* 0x16 */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x07);           /* 0x17 */
    spl07.calib.c10 = (int32_t)(h & 0x0F) << 16 | (int32_t)m << 8 | l;
    spl07.calib.c10 = (spl07.calib.c10 & 0x080000) ? (int32_t)(0xFFF00000 | spl07.calib.c10) : spl07.calib.c10;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x08);           /* 0x18 */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x09);           /* 0x19 */
    spl07.calib.c01 = (int16_t)h << 8 | l;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x0A);           /* 0x1A */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x0B);           /* 0x1B */
    spl07.calib.c11 = (int16_t)h << 8 | l;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x0C);           /* 0x1C */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x0D);           /* 0x1D */
    spl07.calib.c20 = (int16_t)h << 8 | l;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x0E);           /* 0x1E */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x0F);           /* 0x1F */
    spl07.calib.c21 = (int16_t)h << 8 | l;

    h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x10);           /* 0x20 */
    l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x11);           /* 0x21 */
    spl07.calib.c30 = (int16_t)h << 8 | l;

    if (SPL07_Is_003())
    {
        h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x12);       /* 0x22 */
        l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x13);       /* 0x23 */
        spl07.calib.c31 = (int16_t)h << 4 | l >> 4;
        spl07.calib.c31 = (spl07.calib.c31 & 0x0800) ? (int16_t)(0xF000 | spl07.calib.c31) : spl07.calib.c31;

        h = SPA06_Read_Reg(SPA06_Reg_COEF + 0x13);       /* 0x23 */
        l = SPA06_Read_Reg(SPA06_Reg_COEF + 0x14);       /* 0x24 */
        spl07.calib.c40 = (int16_t)((h & 0x0F) << 8) | l;
        spl07.calib.c40 = (spl07.calib.c40 & 0x0800) ? (int16_t)(0xF000 | spl07.calib.c40) : spl07.calib.c40;
    }
}

//配置采样率+过采样(官方 spl07_003_rateset)
// iSensor: 0=气压 1=温度; u8SmplRate: 1/2/4/8/16/32/64/128(Hz)
// u8OverSmpl: 1/2/4/8/16/32/64/128(x)
static void SPL07_Rateset(uint8_t iSensor, uint8_t u8SmplRate, uint8_t u8OverSmpl)
{
    uint8_t reg = 0;
    int32_t i32kPkT = 0;

    switch (u8SmplRate)
    {
        case 2:   reg |= (1 << 4); break;
        case 4:   reg |= (2 << 4); break;
        case 8:   reg |= (3 << 4); break;
        case 16:  reg |= (4 << 4); break;
        case 32:  reg |= (5 << 4); break;
        case 64:  reg |= (6 << 4); break;
        case 128: reg |= (7 << 4); break;
        case 1:
        default:  break;
    }
    switch (u8OverSmpl)
    {
        case 2:   reg |= 1;  i32kPkT = 1572864;  break;
        case 4:   reg |= 2;  i32kPkT = 3670016;  break;
        case 8:   reg |= 3;  i32kPkT = 7864320;  break;
        case 16:  i32kPkT = 253952;  reg |= 4;   break;
        case 32:  i32kPkT = 516096;  reg |= 5;   break;
        case 64:  i32kPkT = 1040384; reg |= 6;   break;
        case 128: i32kPkT = 2088960; reg |= 7;   break;
        case 1:
        default:  i32kPkT = 524288;  break;
    }

    if (iSensor == SPL07_PRESSURE_SENSOR)
    {
        spl07.i32kP = i32kPkT;
        SPA06_Write_Reg(SPA06_Reg_PRS_CFG, reg);
        reg = SPA06_Read_Reg(SPA06_Reg_CFG_REG);
        SPA06_Write_Reg(SPA06_Reg_CFG_REG,
                        (u8OverSmpl > 8) ? (reg | 0x04) : (reg & (uint8_t)~0x04));
    }
    if (iSensor == SPL07_TEMPERATURE_SENSOR)
    {
        spl07.i32kT = i32kPkT;
        /* 0x11 用内部温度; 其它用MEMS温度(官方: 温度寄存器bit7) */
        if (SPL07_Is_003())
            SPA06_Write_Reg(SPA06_Reg_TMP_CFG, reg);
        else
            SPA06_Write_Reg(SPA06_Reg_TMP_CFG, reg | 0x80);
        reg = SPA06_Read_Reg(SPA06_Reg_CFG_REG);
        SPA06_Write_Reg(SPA06_Reg_CFG_REG,
                        (u8OverSmpl > 8) ? (reg | 0x08) : (reg & (uint8_t)~0x08));
    }
}

//启动连续测量: mode 1=仅气压 2=仅温度 3=气压+温度(官方连续=bit2置1 + mode)
static void SPL07_Start_Continuous(uint8_t mode)
{
    SPA06_Write_Reg(SPA06_Reg_MEAS_CFG, (uint8_t)(mode + 4));
}

//补偿温度(官方 spl07_003_get_temperature), 单位°C
static float SPL07_Get_Temperature(void)
{
    float fTsc;
    fTsc = spl07.i32rawTemperature / (float)spl07.i32kT;
    return spl07.calib.c0 * 0.5f + spl07.calib.c1 * fTsc;
}

//补偿气压(官方 spl07_003_get_pressure), 单位Pa
static float SPL07_Get_Pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = spl07.i32rawTemperature / (float)spl07.i32kT;
    fPsc = spl07.i32rawPressure   / (float)spl07.i32kP;

    qua2 = spl07.calib.c10 + fPsc * (spl07.calib.c20 +
           fPsc * (spl07.calib.c30 + fPsc * spl07.calib.c40));

    if (SPL07_Is_003())
        qua3 = fTsc * fPsc * (spl07.calib.c11 +
               fPsc * (spl07.calib.c21 + fPsc * spl07.calib.c31));
    else
        qua3 = fTsc * fPsc * (spl07.calib.c11 + fPsc * spl07.calib.c21);

    fPCompensate = spl07.calib.c00 + fPsc * qua2 +
                   fTsc * spl07.calib.c01 + qua3;
    return fPCompensate;
}

//24bit原始数据符号扩展
static int32_t SPL07_SignExtend24(uint32_t value)
{
    if (value & 0x800000) value |= 0xFF000000;
    return (int32_t)value;
}

/* ============================ 对外接口 ============================== */
//SPA06-003初始化(读ID、标定系数、配置采样并启动连续测量)
void SPA06_Init(void)
{
    uint8_t status;

    // 0. 软复位芯片(寄存器0x0C SOFT_RST=0x9), 清掉之前可能残留的异常状态
    SPA06_Write_Reg(SPA06_Reg_RESET, 0x09);
    osDelay(15);

    // 1. 等待上电自检与标定系数就绪(COEF_RDY + SENSOR_RDY)
    for (uint8_t i = 0; i < 100; i++)
    {
        status = SPA06_Read_Reg(SPA06_Reg_MEAS_CFG);
        if ((status & 0xC0) == 0xC0)
            break;
        osDelay(2);
    }

    // 2. 读取芯片ID(0x11 即 SPA06-003)
    SPL07_Read_Chip_Id();
    spl07.i32rawPressure    = 0;
    spl07.i32rawTemperature = 0;

    // 3. 读取标定系数(逐字节读取)
    SPL07_Get_Calib_Param();

    // 4. 配置采样率/过采样: 气压16Hz/16x, 温度1Hz/8x  (降噪组合)
    //    说明: 过采样越高噪声越小, 但单次转换时间越长, 采样率需相应降低,
    //    否则转换跟不上会读到接近0(此时把采样率再降一档即可)。
    //    噪声更低可试: 气压 8Hz/32x, 温度 1Hz/16x;
    //    更新率优先可试: 气压 32Hz/8x(原始配置)。
    SPL07_Rateset(SPL07_PRESSURE_SENSOR,    16, 16);
    SPL07_Rateset(SPL07_TEMPERATURE_SENSOR, 1, 8);

    // 5. 启动后台连续测量(气压+温度)
    SPL07_Start_Continuous(3);
}

//SPA06-003读全部数据(气压+温度+未滤波海拔+相对高度)
void SPA06_Read_Data(void)
{
    uint8_t buf[6] = {0};
    uint32_t rawP = 0;
    uint32_t rawT = 0;
    uint8_t new_data = 0;

    // 按官方参考驱动分两次读: 气压0x00~0x02 与 温度0x03~0x05
    // 说明: 部分SPA06-003一次6字节跨通道连读时气压段会读到0(温度正常),
    //       分开成两个3字节事务读才正常。
    SPA06_Read_RegList(SPA06_Reg_PSR_B2, 3, &buf[0]);
    SPA06_Read_RegList(SPA06_Reg_TMP_B2, 3, &buf[3]);

    rawP = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    rawT = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    spl07.i32rawPressure    = SPL07_SignExtend24(rawP);
    spl07.i32rawTemperature = SPL07_SignExtend24(rawT);

    SPA06.Raw_Pressure = spl07.i32rawPressure;
    SPA06.Raw_Temp     = spl07.i32rawTemperature;

    // 软件轮询可能快于传感器更新, 寄存器有刷新才判为新样本
    if ((SPA06.Raw_Pressure != SPA06_LastRawP) || (SPA06.Raw_Temp != SPA06_LastRawT))
        new_data = 1;
    SPA06_LastRawP = SPA06.Raw_Pressure;
    SPA06_LastRawT = SPA06.Raw_Temp;

    // 有新样本才做补偿计算(官方公式)
    if (new_data && (SPA06.Raw_Pressure != 0))
    {
        SPA06.Temperature = SPL07_Get_Temperature();
        SPA06.Pressure    = SPL07_Get_Pressure();

        // 国际气压测高公式
        if (SPA06.Pressure > 0.0f)
        {
            double ratio = pow((double)SPA06.Pressure / (double)SPA06_SEA_PRESSURE, 1.0 / 5.255);
            SPA06.Altitude = (float)(44330.0 * (1.0 - ratio));
        }
        else
        {
            SPA06.Altitude = 0.0f;
        }
    }   // 有新样本才更新以上输出, 否则保持上一帧数值

    // 相对高度: 以上电首个有效海拔为零点, 未滤波
    if (new_data && (SPA06.Pressure > 0.0f))
    {
        if (!SPA06_Height_Ready)
        {
            SPA06_Height_Zero  = SPA06.Altitude;   // 首样本建零点
            SPA06_Height_Ready = 1;
        }
        SPA06.Relative_Height = SPA06.Altitude - SPA06_Height_Zero;
        SPA06.Relative_Height_mm = (int32_t)(SPA06.Relative_Height * 1000.0f +
                                             (SPA06.Relative_Height >= 0.0f ? 0.5f : -0.5f));
    }
}

//SPA06-003相对高度零点重置接口: 以当前(未滤波)海拔为新零点(相对高度归零)
void SPA06_Reset_Height(void)
{
    if (!SPA06_Height_Ready)
        return;

    SPA06_Height_Zero        = SPA06.Altitude;
    SPA06.Relative_Height    = 0.0f;
    SPA06.Relative_Height_mm = 0;
}


