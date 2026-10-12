#ifndef __SPA06_H__
#define __SPA06_H__

#include "main.h"

//SPA06-003 I2C 从机地址(SDO悬空/接高=0x77, SDO接地=0x76)
#define SPA06_I2C_ADDR       0x77

//补偿比例系数(手册Table 4, 按实际配置的过采样率选取)
#define SPA06_KP_8X          7864320

#define SPA06_KT_1X          524288

//气压标准大气压(单位Pa)
#define SPA06_SEA_PRESSURE   101325.0f

//SPA06-003 寄存器地址
typedef enum
{
    SPA06_Reg_PSR_B2      = 0x00, // 气压数据最高字节
    SPA06_Reg_PSR_B1      = 0x01, // 气压数据中间字节
    SPA06_Reg_PSR_B0      = 0x02, // 气压数据最低字节
    SPA06_Reg_TMP_B2      = 0x03, // 温度数据最高字节
    SPA06_Reg_TMP_B1      = 0x04, // 温度数据中间字节
    SPA06_Reg_TMP_B0      = 0x05, // 温度数据最低字节
    SPA06_Reg_PRS_CFG     = 0x06, // 气压测量配置(测量速率+过采样)
    SPA06_Reg_TMP_CFG     = 0x07, // 温度测量配置
    SPA06_Reg_MEAS_CFG    = 0x08, // 工作模式与状态
    SPA06_Reg_CFG_REG     = 0x09, // 中断/FIFO/移位配置
    SPA06_Reg_INT_STS     = 0x0A, // 中断状态
    SPA06_Reg_FIFO_STS    = 0x0B, // FIFO状态
    SPA06_Reg_RESET       = 0x0C, // 软复位/FIFO清空
    SPA06_Reg_ID          = 0x0D, // 产品ID(0x11)
    SPA06_Reg_COEF        = 0x10, // 标定系数起始地址(0x10~0x24)
} SPA06_Reg_EnumTypedef;

//SPA06-003 标定系数
typedef struct
{
    int32_t c00;    // 20bit
    int32_t c10;    // 20bit
    int16_t c20;    // 16bit
    int16_t c30;    // 16bit
    int16_t c40;    // 12bit
    int16_t c01;    // 16bit
    int16_t c11;    // 16bit
    int16_t c21;    // 16bit
    int16_t c31;    // 12bit
    int16_t c0;     // 12bit
    int16_t c1;     // 12bit
} SPA06_Coef_Typedef;

//SPA06-003 数据结构体
typedef struct
{
    float Pressure;         // 气压 (Pa)
    float Temperature;      // 温度 (°C)
    float Altitude;         // 气压计算海拔, 未滤波 (m)
    int32_t Raw_Pressure;   // 原始24bit气压
    int32_t Raw_Temp;       // 原始24bit温度

    float Relative_Height;  // 相对高度 (m), 以最近一次重置/上电首样本为零点, 未滤波
    int32_t Relative_Height_mm; // 相对高度 (mm)
} SPA06_StructTypedef;

//SPA06-003数据结构体
extern SPA06_StructTypedef SPA06;

//SPA06-003初始化(读ID、标定系数并配置连续测量)
void SPA06_Init(void);
//SPA06-003读全部数据(气压+温度+未滤波海拔+相对高度)
void SPA06_Read_Data(void);
//SPA06-003相对高度零点重置(以当前未滤波海拔为新零点, 相对高度归零)
void SPA06_Reset_Height(void);

//SPA06-003读单个寄存器
uint8_t SPA06_Read_Reg(uint8_t Reg);
//SPA06-003读多个寄存器
void SPA06_Read_RegList(uint8_t Reg, uint8_t Num, uint8_t *Read_Data);
//SPA06-003写单个寄存器
void SPA06_Write_Reg(uint8_t Reg, uint8_t Write_Data);

#endif
