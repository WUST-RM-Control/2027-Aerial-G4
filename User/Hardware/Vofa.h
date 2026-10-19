#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"

#define VOFA_CH_MAX     16      /* 最多 16 个 float 通道(VOFA+ justfloat 格式) */

/*============================================================================
 *  发送节拍: 独立于飞行主循环(TIM6), 由 TIM7 单独产生
 *    频率 = 170MHz / (PSC+1) / (ARR+1), PSC/ARR 由 Vofa_Set_Send_Hz() 算出,
 *    只改"多久发一帧", 与串口波特率无关。
 *    改频率两种方式:
 *      1) 改下面的宏(编译期默认值, 上电即生效);
 *      2) 运行中调用 Vofa_Set_Send_Hz(hz) (调试器里调也可以)。
 *    例: 4000 -> 4kHz(250us), 2000 -> 2kHz(500us), 1000 -> 1kHz(1ms)
 *    TIM7 只负责"到点了"(发信号量), 组帧和发送仍在 Communicate 任务里做。
 *
 *  实际的发送次数看 Vofa_Tick_Count, 真正发出去的帧数看 Vofa_Send_Count
 *  (只有当帧间隔比 UART 发送耗时还短时两者才会不一致, 差值记在 Vofa_Drop_Count)。
 *===========================================================================*/
#define VOFA_SEND_HZ        1000u

void Vofa_JF_Add_Data(uint8_t Index, float Data);
void Vofa_JF_Send(void);

/* ---- 定时发送(TIM7 节拍) ---- */
void Vofa_Tick_Init(void);          /* 建信号量 + 启动 TIM7(在任务里调用) */
void Vofa_Wait_Tick(void);          /* 阻塞到下一个 TIM7 节拍             */
void Vofa_Set_Send_Hz(uint32_t Hz); /* 运行中改发送频率(1Hz~85MHz)        */
void Vofa_Timer_ISR(void);          /* TIM7_DAC_IRQHandler 里调用         */

/* 上一帧 DMA 还没发完被丢掉(未被丢弃时恒为 0) */
extern volatile uint32_t Vofa_Drop_Count;
extern volatile uint32_t Vofa_Send_Count;   /* 真正启动 DMA 的帧数      */
extern volatile uint32_t Vofa_Tick_Count;   /* TIM7 节拍数(名义发送频率) */

#endif
