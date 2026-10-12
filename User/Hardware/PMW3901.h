#ifndef __PMW3901__
#define __PMW3901__

#include "main.h"

//PMW3901 寄存器地址
typedef enum
{
    PMW3901_Reg_Product_ID    = 0x00, // 产品ID,应为0x49
    PMW3901_Reg_Revision_ID   = 0x01, // 版本ID
    PMW3901_Reg_Motion        = 0x02, // bit7:本帧是否发生位移
    PMW3901_Reg_Delta_X_L     = 0x03, // X方向位移(低字节)
    PMW3901_Reg_Delta_X_H     = 0x04, // X方向位移(高字节)
    PMW3901_Reg_Delta_Y_L     = 0x05, // Y方向位移(低字节)
    PMW3901_Reg_Delta_Y_H     = 0x06, // Y方向位移(高字节)
    PMW3901_Reg_SQUAL         = 0x07, // 表面质量(0~169,>~50为可靠)
    PMW3901_Reg_RawData_Sum   = 0x08, // 原始数据总和
    PMW3901_Reg_Maximum_Raw   = 0x09, // 原始数据最大值
    PMW3901_Reg_Minimum_Raw   = 0x0A, // 原始数据最小值
    PMW3901_Reg_Shutter_Upper = 0x0B, // 曝光时间(上)
    PMW3901_Reg_Shutter_Lower = 0x0C, // 曝光时间(下)
    PMW3901_Reg_Motion_Burst  = 0x16, // 突发读:一次性读出全部运动数据
} PMW3901_Reg_EnumTypedef;

//PMW3901数据结构体(突发读返回, 与手册寄存器顺序一致)
typedef __PACKED_STRUCT
{
    uint8_t Motion;       // bit7 = 本帧是否发生位移
    uint8_t Observation;  // 观测/光照状态
    int16_t Delta_X;      // X方向位移(计数)
    int16_t Delta_Y;      // Y方向位移(计数)
    uint8_t SQUAL;        // 表面质量
    uint8_t RawData_Sum;  // 原始数据总和
    uint8_t Maximum_Raw;  // 原始数据最大值
    uint8_t Minimum_Raw;  // 原始数据最小值
    uint16_t Shutter;     // 曝光时间(注意字节序需交换)
} PMW3901_StructTypedef;

//PMW3901数据结构体(原始计数, 上层积分成位移)
extern PMW3901_StructTypedef PMW3901;

//PMW3901初始化(内部会把SPI1切到模式3, 波特率分频128)
void PMW3901_Init(void);
//PMW3901突发读全部运动数据
void PMW3901_Read_Data(void);

//PMW3901读单个寄存器
uint8_t PMW3901_Read_Reg(PMW3901_Reg_EnumTypedef Reg);
//PMW3901写单个寄存器
void PMW3901_Write_Reg(PMW3901_Reg_EnumTypedef Reg, uint8_t Write_Data);

#endif
