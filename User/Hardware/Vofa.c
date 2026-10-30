#include "Vofa.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "tim.h"

uint8_t Data_Num = 0;
float Vofa_Data[VOFA_CH_MAX + 2];      /* 通道 + 帧尾(0x7f800000), 末尾留 2 个防越界 */

/* DMA 发送缓冲: 与 Vofa_Data 分开, 避免"上一帧还在发, 下一帧已经改写数据" */
static uint8_t Vofa_Tx_Buf[4 * (VOFA_CH_MAX + 2)];

volatile uint32_t Vofa_Drop_Count = 0; /* 上一帧还没发完而被丢弃的帧数(调试用) */
volatile uint32_t Vofa_Send_Count = 0; /* 真正启动 DMA 的帧数                      */
volatile uint32_t Vofa_Tick_Count = 0; /* TIM7 节拍数(名义 1kHz)                   */

/* TIM7 节拍信号量: ISR 里 Give, Communicate 任务里 Take(二值信号量, 最多挂 1 个) */
static SemaphoreHandle_t Vofa_Sem = NULL;

#define VOFA_TIM_CLK_HZ   170000000u   /* TIM7 挂在 APB1, 定时器时钟 = 170MHz */
#define VOFA_TIM_PSC_1MHZ 169u         /* (169+1) 分频 -> 1MHz 计数时钟 */

/* 发送口: **USART2**(调试口 U26 6P, PA15/PB3, 已配 4 Mbaud + TX DMA1_Channel4)
 * 注: USART1 是遥测口(U38 4P), 保持 115200, 不被 VOFA 占用 */
#define Vofa_Send(Data, Length) HAL_UART_Transmit_DMA(&huart2, (Data), (uint16_t)(Length))

/*============================================================================
 *  TIM7 节拍: 独立于 TIM6(飞行主循环), 频率单独可调
 *===========================================================================*/
void Vofa_Set_Send_Hz(uint32_t Hz)
{
    uint32_t psc = VOFA_TIM_PSC_1MHZ;   /* 默认 1MHz 计数时钟, 与 .ioc 的 169 一致 */
    uint32_t arr;
    uint32_t clk;
    uint8_t  running;

    if (Hz == 0u) Hz = 1u;
    if (Hz > VOFA_TIM_CLK_HZ / 2u) Hz = VOFA_TIM_CLK_HZ / 2u;

    /* 16 位计数器放不下就先加大预分频 */
    for (;;)
    {
        clk = VOFA_TIM_CLK_HZ / (psc + 1u);
        arr = clk / Hz;
        if (arr <= 65536u) break;
        psc++;
    }
    if (arr == 0u) arr = 1u;

    running = (uint8_t)((htim7.Instance->CR1 & TIM_CR1_CEN) != 0u);
    if (running) __HAL_TIM_DISABLE(&htim7);      /* 运行中改 PSC/ARR 要先停表 */

    htim7.Init.Prescaler = psc;
    htim7.Init.Period    = arr - 1u;
    __HAL_TIM_SET_PRESCALER(&htim7, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim7, arr - 1u);
    __HAL_TIM_SET_COUNTER(&htim7, 0u);

    if (running) __HAL_TIM_ENABLE(&htim7);        /* 注意: 不能用 Base_Start, 否则会重开中断 */
}

void Vofa_Timer_ISR(void)
{
    BaseType_t woken = pdFALSE;

    Vofa_Tick_Count++;

    if (Vofa_Sem != NULL)
    {
        xSemaphoreGiveFromISR(Vofa_Sem, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

void Vofa_Tick_Init(void)
{
    if (Vofa_Sem == NULL)
    {
        Vofa_Sem = xSemaphoreCreateBinary();
    }

    Vofa_Set_Send_Hz(VOFA_SEND_HZ);   /* 以宏为准: 即使 CubeMX 重新生成过 tim.c 也能对上 */
    HAL_TIM_Base_Start_IT(&htim7);
}

void Vofa_Wait_Tick(void)
{
    if (Vofa_Sem != NULL)
    {
        xSemaphoreTake(Vofa_Sem, portMAX_DELAY);
    }
    else
    {
        osDelay(1);     /* 信号量没建起来(不该发生): 兜底定时, 不空转 */
    }
}

void Vofa_JF_Add_Data(uint8_t Index, float Data)
{
    if (Index >= VOFA_CH_MAX) return;          /* 防越界: 帧尾就写在第 Index+1 个 float */

    Data_Num = MAX(Index, Data_Num);
    Vofa_Data[Index] = Data;
}

/**
 * @brief  justfloat 帧发送(改为 DMA, 不占 CPU)
 * @note   原来用 HAL_UART_Transmit 阻塞发送: 1Mbps 下 40 字节约占 400us,
 *         每 10ms 发一帧就等于烧掉 4% CPU(而且是纯忙等)。
 *         现在发送口是 USART2(调试口), 已提到 4Mbps, 且用 DMA, CPU 只做一次 memcpy。
 *
 *         为什么用独立缓冲而不是直接发 Vofa_Data:
 *         若下一帧在 DMA 还没发完时改写 Vofa_Data, 发出去的会是半新半旧的混合数据。
 *         独立缓冲 + "上一帧没发完就丢掉这一帧" 对遥测完全够用(丢帧无所谓, 不能阻塞)。
 *
 *         为什么判断 huart.gState 而不是 HAL_UART_GetState():
 *         USART2 的 RX 一直挂在 HAL_UARTEx_ReceiveToIdle_DMA 上(RxState=BUSY_RX),
 *         而 HAL_UART_GetState() 返回 gState|RxState, 永远不等于 READY -> 一帧都发不出去。
 */
void Vofa_JF_Send(void)
{
    uint16_t Len = (uint16_t)(4u * (Data_Num + 2u));

    ((uint32_t *)Vofa_Data)[Data_Num + 1] = 0x7f800000;   /* justfloat 帧尾 */
    memcpy(Vofa_Tx_Buf, Vofa_Data, Len);

    if (huart2.gState == HAL_UART_STATE_READY)
    {
        if (Vofa_Send(Vofa_Tx_Buf, Len) == HAL_OK) Vofa_Send_Count++;
        else                                      Vofa_Drop_Count++;
    }
    else
    {
        Vofa_Drop_Count++;      /* 上一帧还在发: 丢掉本帧, 绝不在这里阻塞 */
    }

    Data_Num = 0;
}
