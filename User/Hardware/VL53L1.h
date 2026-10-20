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
    uint8_t  Updated;      // 1=本次调用取到了"新的"测量样本(新样本标志, 供差分用)
} VL53L1_StructTypedef;

//VL53L1数据结构体
extern VL53L1_StructTypedef VL53L1;

//VL53L1初始化(写默认配置+一次自检, 耗时较长; I2C3由main.c的MX_I2C3_Init()提供)
void VL53L1_Init(void);
//VL53L1读距离: 非阻塞轮询(未就绪立即返回0); 返回1=本次取到新样本
// 调用方应用"两次新样本的时间差"(约33ms)做差分, 不能用任务周期(10ms)
uint8_t VL53L1_Read_Data(void);

/* 新样本计数(调试用): 读两次求差即可得到实际的 TOF 更新率 */
extern volatile uint32_t VL53L1_New_Count;

/* ---- 调试量: EmberProbe 直接读写, 不需要改代码/重烧 ---- */
extern volatile uint32_t VL53L1_Dbg_InterMeas;   /* 0x006C 测量间隔计数值(≈39.77 counts/ms), 写后立即生效 */
extern volatile uint16_t VL53L1_Dbg_RangeStatus; /* 0x0089 低5位, 0=正常 */
extern volatile uint16_t VL53L1_Dbg_SignalRate;  /* 0x0098 峰值信号率, 9.7 定点(MCPS=值/128) */
extern volatile uint16_t VL53L1_Dbg_AmbientRate; /* 0x0090 环境光率, 9.7 定点(MCPS=值/128) */
extern volatile uint16_t VL53L1_Dbg_EffSpads;    /* 0x008C DSS 有效 SPAD, 8.8 定点(个数=值/256) */

/* ---- 以下为移植自文档 vl53l1.c/h 的原始接口 ---- */
uint16_t vl53l1_readdistance(void);   // 原始 readdistance 接口
uint16_t getDistance(void);           // 读 0x0096 距离(mm); >4000 返回 65535
void     setDistanceModeShort(void);  // 短程 0~1.3m
void     setDistanceModeLong(void);   // 长程 0~4m (初始化默认)
void     setTimingBudgetInMs(uint16_t ms); // 15/20/33/50/100/200/500ms
int16_t  getOffset(void);             // 读偏移校正值(mm)
void     setOffset(int16_t mm);       // 写偏移校正值(mm)

#endif
