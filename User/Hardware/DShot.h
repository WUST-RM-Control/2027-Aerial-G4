#ifndef __DSHOT_H__
#define __DSHOT_H__

#include "main.h"

/*==============================================================================
 *  DShot600 电调输出驱动 (TIM2 CH1~CH4 + DMA)
 *  引脚: PA0 = TIM2_CH1, PA1 = TIM2_CH2, PA2 = TIM2_CH3, PA3 = TIM2_CH4
 *
 *  定时器时序(CubeMX 配置):
 *      TIM2 时钟      : 170MHz (APB1 Timer Clock)
 *      计数频率       : 170MHz / (13+1) = 12.143MHz
 *      1 个 PWM 周期  : 20 个计数 = 1.647us  ->  607kHz ≈ DShot600 (600kHz, 快 1.2%)
 *      一个 PWM 周期 = 一个 bit, 高电平占几个计数决定该 bit 是 0 还是 1
 *          '1' -> 15 计数 ≈ 1.235us (标准 1.250us, 75%)
 *          '0' ->  8 计数 ≈ 0.659us (标准 0.625us, 40%)
 *      一帧 = 1 拍同步低 + 16bit 数据 + 4 拍帧间隔 = 21 个 PWM 周期 ≈ 34.6us
 *
 *  DMA: 4 路各占一个 DMA 通道(DMA1_CH6/CH7/CH8 + DMA2_CH1),
 *       触发源 = TIM2_CHx 的比较(CC)事件, 每来一次事件搬一个 CCR 值 = 一个 bit
 *      ⚠ 方向必须是 存储器 -> 外设(MEMORY_TO_PERIPH), 否则发不出任何东西
 *      ⚠ DMA 数据宽度必须是 HALFWORD(CCR 是半字缓冲), 见 .ioc 的 TIM2_CHx 配置
 *============================================================================*/

#define DSHOT_MOTOR_NUM     4       /* 电机(电调)路数 */
#define DSHOT_BUF_LEN       21      /* 1(同步低) + 16(数据) + 4(帧间隔) */

/* 高电平计数值: 20 个计数 = 1 bit, 按 DShot 规范的 3/8 与 3/4 占空比取整
 *  '0' 取 7 而不取 8: 7 计数 = 0.576us 更贴近规范的 0.625us,
 *  而且离电调的判决门限(≈50%, 0.82us)更远 —— DShot 里 16 位全 1 恰好是
 *  合法的"满油 2047"帧, '0' 被误判成 '1' 的现象就是"一使能就满油门" */
#define DSHOT_CNT_BIT_1     15      /* '1': 15/20 = 75%  */
#define DSHOT_CNT_BIT_0     7       /* '0':  7/20 = 35%  */

/* DShot 油门值: 0 = 停转/上锁, 1~47 = 特殊命令, 48~2047 = 油门(48 对应 0%) */
#define DSHOT_THROTTLE_MIN  48
#define DSHOT_THROTTLE_MAX  2047

void DShot_Init(void);                                      /* 初始化输出(TIM2 启动 + DMA) */
void DShot_Set_Value(uint8_t Motor_ID, uint16_t Value);     /* 写帧缓冲(不立即发送) */
void DShot_Set_Duty(uint8_t Motor_ID, float Duty_100);      /* 占空比 0~100 -> 油门值 */
void DShot_Send(void);                                      /* 4 路同时发一帧 */

#endif
