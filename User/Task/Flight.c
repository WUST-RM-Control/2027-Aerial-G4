/**
 * @file  Flight.c
 * @brief 飞行主循环: TIM6 4kHz 节拍 -> 二值信号量 -> 单一任务里顺序执行
 *
 *  为什么用二值信号量而不是任务通知:
 *    1) 不需要 xTaskGetCurrentTaskHandle()
 *       (FreeRTOSConfig.h 里没有开 INCLUDE_xTaskGetCurrentTaskHandle);
 *    2) 二值信号量天然"最多挂 1 个": 某一拍超时了, 下一拍醒来只会执行一次,
 *       不会出现"攒了 3 个信号连跑 3 拍补课"的雪崩。
 *
 *  节拍精度:
 *    TIM6 由硬件产生 250us 周期, 抖动 = 中断响应 + 上下文切换(约 2~5us),
 *    比 osDelay(1) 的 tick 抖动小一个数量级。
 */
#include "Flight.h"

#include "FreeRTOS.h"
#include "semphr.h"

#include "tim.h"
#include "INS.h"
#include "Control.h"

static SemaphoreHandle_t Flight_Sem = NULL;

volatile uint32_t Flight_LoopCount      = 0;
volatile float    Flight_LoopTime_us    = 0.0f;
volatile float    Flight_LoopTimeMax_us = 0.0f;
volatile uint32_t Flight_LoopOverRun    = 0;

/*============================================================================
 *  ISR 侧: 只发信号量, 不做任何计算
 *  调用点在 stm32g4xx_it.c 的 TIM6_DAC_IRQHandler 的 USER CODE 区
 *===========================================================================*/
void Flight_Timer_ISR(void)
{
    BaseType_t woken = pdFALSE;

    if (Flight_Sem != NULL)
    {
        xSemaphoreGiveFromISR(Flight_Sem, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/*============================================================================
 *  任务主体
 *    初始化只做一次, 之后每 250us 醒一次跑一拍
 *===========================================================================*/
void Flight_Task(void const *argument)
{
    uint32_t t0, t1;
    float    used_us;

    (void)argument;

    Flight_Sem = xSemaphoreCreateBinary();

    /* 顺序有讲究: INS 先起来, Control_Init 里要读 INS.YawTotalAngle 做航向基准 */
    INS_Init_All();
    Control_Init();

    /* 初始化完成后再开节拍, 避免中断在初始化期间就开始塞信号量 */
    Flight_LoopCount = 0;
    HAL_TIM_Base_Start_IT(&htim6);

    for (;;)
    {
        xSemaphoreTake(Flight_Sem, portMAX_DELAY);

        t0 = DWT->CYCCNT;
        INS_Update();           /* 读IMU -> 校准 -> 陀螺滤波 -> EKF -> 姿态 */
        Control_Update();       /* 摇杆 -> 姿态环 -> 角速度环 -> 混控 -> DShot */
        t1 = DWT->CYCCNT;

        /* 单拍耗时(us): 250us 的预算还剩多少, 直接看这里 */
        used_us = (float)(t1 - t0) * (1000000.0f / (float)SystemCoreClock);
        Flight_LoopTime_us = used_us;
        if (used_us > Flight_LoopTimeMax_us) Flight_LoopTimeMax_us = used_us;
        if (used_us > (1000000.0f / (float)FLIGHT_LOOP_HZ)) Flight_LoopOverRun++;
        Flight_LoopCount++;
    }
}
