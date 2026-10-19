#include "DShot.h"
#include "tim.h"

/*==============================================================================
 *  DShot600 输出驱动 (TIM4 CH1~CH4 + DMA)
 *
 *  实现思路:
 *   1) 一个 PWM 周期 = 一个 bit, bit 的值由"高电平持续几个计数"决定。
 *      每个 bit 发完时(CNT 追上 CCR)会产生一次 CC 事件, 用它触发 DMA,
 *      把下一个 bit 的 CCR 值搬进 TIM4->CCRx。一帧 21 个 bit = 21 次搬运。
 *
 *   2) CCR 预装载(OCxPE)必须打开: DMA 是在高电平结束那一刻写入新值,
 *      若没有预装载, 新值会立刻改变输出电平(例如 '0' 后面跟 '1' 时会提前拉高),
 *      bit 时序就被破坏; 打开预装载后新值只在 update(周期边界)生效,
 *      每个 bit 都是完整的一拍, 高电平宽度严格等于 CCR 值。
 *
 *   3) 帧首固定加 1 拍低电平做同步: DMA 的第一次搬运落在周期的哪个位置
 *      是不确定的, 用 1 拍固定的低电平兜底, 保证后面 16 位数据永远连续、不错位。
 *
 *   4) 缓冲末尾 4 个 0: CCR=0 -> 输出保持低, 即 DShot 帧间隔(规范要求 ≥2us)。
 *      帧发完后 DMA 停止, CCR 停在 0, 信号线保持低电平(空闲态)。 *
 *  5) 启动 DMA 不走 HAL_DMA_Start_IT, 直接写 DMA 寄存器(见 DShot_DMA_Start):
 *      通道配置(方向/自增/位宽/优先级/请求源)已经由 MX_DMA_Init + HAL_DMA_Init 做好,
 *      每帧只需要重装 CPAR/CMAR/CNDTR 再置 EN。
 *      收益: 4 路启动从 6~8us 降到 ~1us, 而且不开传输完成中断
 *            -> 4kHz 下每秒少 16000 次 DMA1_CH6/7/8 + DMA2_CH1 中断(~2~4% CPU 和抖动)。
 *      普通模式下传输结束硬件自动清 EN, 下一帧直接重装即可。 *============================================================================*/

/* tim.h 里没有导出这几个 DMA 句柄(CubeMX 只生成了实例定义), 这里自行声明 */
extern DMA_HandleTypeDef hdma_tim4_ch1;
extern DMA_HandleTypeDef hdma_tim4_ch2;
extern DMA_HandleTypeDef hdma_tim4_ch3;
extern DMA_HandleTypeDef hdma_tim4_ch4;

/* 每路一帧要搬的 CCR 值(半字), DMA 直接从内存搬到 TIM4->CCRx */
static uint16_t DShot_Buf[DSHOT_MOTOR_NUM][DSHOT_BUF_LEN];

/**
 * @brief  启动一路 DMA(直接写寄存器)
 * @param  DMA_Channel 由 hdma_tim4_chX.Instance 给出(DMA1_CH6/7/8, DMA2_CH1)
 * @param  Src         内存源地址(本路 CCR 缓冲)
 * @param  Dst         外设目标地址(&htim4.Instance->CCRx)
 * @note   不调用 HAL_DMA_Start_IT 的原因见文件头说明 5)。
 *         不使能任何 DMA 中断 -> HAL_DMA_IRQHandler / 旧的回调都不再被调用。
 */
static void DShot_DMA_Start(DMA_Channel_TypeDef *DMA_Channel, uint32_t Src, uint32_t Dst)
{
    DMA_Channel->CCR  &= ~DMA_CCR_EN;          /* 上一帧若还没结束(异常情况)先停掉 */
    while (DMA_Channel->CCR & DMA_CCR_EN) { }  /* 等硬件确认已停止(HAL_DMA_Abort 同款做法) */
    DMA_Channel->CPAR  = Dst;
    DMA_Channel->CMAR  = Src;
    DMA_Channel->CNDTR = DSHOT_BUF_LEN;        /* 21 个半字 = 21 个 bit */
    DMA_Channel->CCR  |= DMA_CCR_EN;
}

/**
 * @brief  DShot 输出初始化
 * @note   调用前 MX_DMA_Init() / MX_TIM4_Init() 必须已经执行
 */
void DShot_Init(void)
{
    /* 1) 打开 CCR 预装载, 关闭 fast mode(原因见文件头说明) */
    htim4.Instance->CCMR1 |=  (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
    htim4.Instance->CCMR2 |=  (TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE);
    htim4.Instance->CCMR1 &= ~(TIM_CCMR1_OC1FE | TIM_CCMR1_OC2FE);
    htim4.Instance->CCMR2 &= ~(TIM_CCMR2_OC3FE | TIM_CCMR2_OC4FE);

    /* 2) 输出脚翻转速度拉满: CubeMX 生成的是 GPIO_SPEED_FREQ_LOW, 600kHz 下
     *    边沿太缓会把 0.58us 的 '0' 脉冲"拉宽", 电调很容易把它当成 '1'
     *    (16 位全 1 在 DShot 里是合法的满油帧 -> 现象就是"一使能就满油门")。
     *    写在 DShot_Init 里, CubeMX 重新生成也不会被覆盖 */
    {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &gpio);
    }

    /* 3) 启动 4 路 PWM(CCR=0 -> 空闲低电平), 计数器开始跑 */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    /* 4) 先填一帧"停转"(油门值 0), 保证发出去的是合法帧而不是随机波形 */
    for (uint8_t i = 0; i < DSHOT_MOTOR_NUM; i++)
        DShot_Set_Value(i, 0);
}

/**
 * @brief  把 DShot 原始值写进发送缓冲(只改缓冲, 不发送)
 * @param  Motor_ID 0~3
 * @param  Value    0 = 停转/上锁, 1~47 = 特殊命令, 48~2047 = 油门
 */
void DShot_Set_Value(uint8_t Motor_ID, uint16_t Value)
{
    uint16_t Frame;
    uint16_t CRC16;
    uint8_t  i;

    if (Motor_ID >= DSHOT_MOTOR_NUM) return;
    if (Value > DSHOT_THROTTLE_MAX) Value = DSHOT_THROTTLE_MAX;

    /* 16bit 帧: [15:5] = 油门 11 位, [4] = 请求遥测位(0), [3:0] = CRC 4 位 */
    Frame = (uint16_t)(Value << 1);
    CRC16   = (uint16_t)((Frame ^ (Frame >> 4) ^ (Frame >> 8)) & 0x0FU);
    Frame = (uint16_t)((Frame << 4) | CRC16);

    DShot_Buf[Motor_ID][0] = 0;                        /* 帧首: 同步低电平 */

    for (i = 0; i < 16; i++)                           /* MSB 先发 */
    {
        DShot_Buf[Motor_ID][i + 1] = (Frame & (0x8000U >> i)) ? DSHOT_CNT_BIT_1 : DSHOT_CNT_BIT_0;
    }

    for (i = 17; i < DSHOT_BUF_LEN; i++)
    {
        DShot_Buf[Motor_ID][i] = 0;                    /* 帧尾: 拉低做帧间隔 */
    }
}

/**
 * @brief  占空比指令(0~100) -> DShot 油门值(48~2047)
 * @note   0 单独处理: DShot 里 0 是"停转/上锁"命令, 不能映射成 48
 */
void DShot_Set_Duty(uint8_t Motor_ID, float Duty_100)
{
    float Value;

    if (Duty_100 <= 0.0f)
    {
        DShot_Set_Value(Motor_ID, 0);                  /* 停转/上锁 */
        return;
    }
    if (Duty_100 > 100.0f) Duty_100 = 100.0f;

    Value = (float)DSHOT_THROTTLE_MIN
          + Duty_100 * ((float)(DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN) / 100.0f);
    DShot_Set_Value(Motor_ID, (uint16_t)(Value + 0.5f));
}

/**
 * @brief  4 路同时发一帧
 * @note   在控制周期里调用一次: 当前控制环 4kHz(250us) -> DShot 也是 4kHz。
 *         一帧 21 个 bit @607kHz ≈ 34.6us, 占 250us 周期的 14%, 帧间隔充足;
 *         电调靠连续收帧保持"信号正常", 发送频率高只会让更新更及时。
 *         注: 若某些电调不兼容 4kHz 输入, 在 Control_Update 里改成每 2 拍发一次即可
 *             (2kHz 输出), 对控制环无影响。
 */
void DShot_Send(void)
{
    DShot_DMA_Start(hdma_tim4_ch1.Instance, (uint32_t)DShot_Buf[0], (uint32_t)&htim4.Instance->CCR1);
    DShot_DMA_Start(hdma_tim4_ch2.Instance, (uint32_t)DShot_Buf[1], (uint32_t)&htim4.Instance->CCR2);
    DShot_DMA_Start(hdma_tim4_ch3.Instance, (uint32_t)DShot_Buf[2], (uint32_t)&htim4.Instance->CCR3);
    DShot_DMA_Start(hdma_tim4_ch4.Instance, (uint32_t)DShot_Buf[3], (uint32_t)&htim4.Instance->CCR4);

    /* 使能 CC 事件触发 DMA: 每来一次比较事件搬一个 bit */
    __HAL_TIM_ENABLE_DMA(&htim4, TIM_DMA_CC1 | TIM_DMA_CC2 | TIM_DMA_CC3 | TIM_DMA_CC4);
}
