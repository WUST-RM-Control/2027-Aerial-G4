/**
 * @file  Flight.h
 * @brief 飞行主循环节拍: TIM6 4kHz -> 信号量 -> 单一 Flight 任务
 *
 *  一拍(250us)内的顺序固定为:
 *      INS_Update()      读IMU/校准/陀螺滤波/EKF/姿态
 *      Control_Update()  摇杆->姿态环->角速度环->混控->DShot 输出
 *
 *  为什么必须由一个任务串起来:
 *    1kHz 时代 INS_Task 与 Control_Task 各跑各的, 谁先谁后不确定, 控制可能用到
 *    上一拍的姿态; 4kHz(250us)下这点抖动占比更大。合并后顺序确定、无跨任务共享。
 *
 *  为什么不用 osDelay(1) 定时:
 *    FreeRTOS tick = 1kHz, osDelay(1) 最小只能到 1ms, 且调高 tick 会把所有
 *    osDelay(n) 的语义全改掉(Sensor 的 osDelay(20) 会变成 5ms)。
 *    这里保留 1kHz tick 不动, 另用 TIM6 硬件节拍触发。
 */
#ifndef __FLIGHT_H__
#define __FLIGHT_H__

#include "main.h"

/* 飞行主循环频率: TIM6 = 170MHz/(169+1)/(249+1) = 4kHz, 与 tim.c 的 MX_TIM6_Init 对应 */
#define FLIGHT_LOOP_HZ      4000u
#define FLIGHT_LOOP_DT      (1.0f / (float)FLIGHT_LOOP_HZ)     /* 0.25ms */

/* ISR 侧: 只发信号量, 不做任何计算(在 TIM6_DAC_IRQHandler 里调用) */
void Flight_Timer_ISR(void);

/* 任务主体(osThreadDef 入口) */
void Flight_Task(void const *argument);

/* 单拍耗时统计: 调试器/Vofa 直接看, 用来判断 250us 还够不够 */
extern volatile uint32_t Flight_LoopCount;        /* 已执行拍数                    */
extern volatile float    Flight_LoopTime_us;      /* 最近一拍执行耗时(us)          */
extern volatile float    Flight_LoopTimeMax_us;   /* 上电以来最大单拍耗时(us)      */
extern volatile uint32_t Flight_LoopOverRun;      /* 超过 250us 的拍数             */

#endif /* __FLIGHT_H__ */
