/**
 ******************************************************************************
 * @file	DWT.h
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _DWT_H
#define _DWT_H

#include "main.h"
#include "stdint.h"

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

void DWT_Init(uint32_t CPU_Freq_mHz);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_SysTimeUpdate(void);

/* ---- 给 IMU 样本时间戳用: DWT 周期数 <-> 时间 ----
 * 用法: 在 ISR 里 DWT->CYCCNT 记下样本时刻, 任务里算出相邻样本的周期差,
 *       再传给 DWT_Cyc_To_S() 得到真实 dt。
 * 32 位周期数直接做减法(无符号), 回绕自动正确(170MHz 满量程 ≈ 25 s)。 */
uint32_t DWT_GetCpuFreq_Hz(void);          /* CPU 主频(Hz), DWT_Init 时设置 */
float    DWT_Cyc_To_S(uint32_t cycles);     /* 周期数 -> 秒(浮点, 不丢精度) */
uint32_t DWT_Cyc_To_Us(uint32_t cycles);    /* 周期数 -> 微秒(整数, 便于查看) */

extern DWT_Time_t SysTime;

#endif /* BSP_DWT_H_ */
