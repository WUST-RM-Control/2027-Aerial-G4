#ifndef __VL53L1_H__
#define __VL53L1_H__

#include "main.h"   // 内含 custom_bus.h(hi2c3) 与 HAL 头

//VL53L1 I2C 从机地址(8位总线地址 = 0x29 << 1, 与 SPA06 同挂 I2C3)
#define VL53L1_I2C_ADDR_BYTE     0x52

//VL53L1数据结构体
typedef struct
{
    float    Distance_mm;  // 测距结果(mm); 65535 = 无效/超量程
    uint16_t Status;       // 0=读到有效距离; 1=无效/超量程
    uint8_t  Updated;      // 本次 Read_Data 是否取到有效数据
} VL53L1_StructTypedef;

//VL53L1数据结构体
extern VL53L1_StructTypedef VL53L1;

//VL53L1初始化(写默认配置+一次自检, 耗时较长; I2C3由main.c的MX_I2C3_Init()提供)
void VL53L1_Init(void);
//VL53L1读距离(移植驱动原流程: start -> 立即读0x0096 -> stop), 返回1=读到有效距离
uint8_t VL53L1_Read_Data(void);

/* ---- 以下为移植自文档 vl53l1.c/h 的原始接口 ---- */
uint16_t vl53l1_readdistance(void);   // 原始 readdistance 接口
uint16_t getDistance(void);           // 读 0x0096 距离(mm); >4000 返回 65535
void     setDistanceModeShort(void);  // 短程 0~1.3m
void     setDistanceModeLong(void);   // 长程 0~4m (初始化默认)
void     setTimingBudgetInMs(uint16_t ms); // 15/20/33/50/100/200/500ms
int16_t  getOffset(void);             // 读偏移校正值(mm)
void     setOffset(int16_t mm);       // 写偏移校正值(mm)

#endif
