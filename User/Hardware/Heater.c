/**
 ******************************************************************************
 * @file    Heater.c
 * @brief   IMU 恒温加热闭环 + 多重保护(V3 控制板, PA11 = TIM4_CH1)
 *
 *  硬件: PA11 -> 120Ω -> Q1(AO3400) 栅极(10k 下拉), 漏极 -> 5V 上两只 240Ω
 *        并联(等效 120Ω)加热电阻 -> 最大约 0.2W。功率不大, 但一旦软件跑飞
 *        把栅极一直拉高, IMU 会持续被烤, 所以这里所有保护都按"宁可不加热"做。
 *
 *  输出方式(TIM4 单脉冲 OPM):
 *      每个 20ms 控制窗口只触发一次: 计数器从 0 跑到 ARR(=19999, 1MHz),
 *      其中前 CCR1 个计数输出高 -> 一个宽度 = 占空比×20ms 的脉冲, 到 ARR
 *      硬件自动清 CEN 并回到低电平。
 *      => 软件即使完全卡死, 也最多再加热 20ms, 不会"一直加热"。
 *
 *  控制: 目标 40℃, 前馈 + PI, 抗积分饱和 + 占空比斜率限制。
 *
 *  保护优先级(每一拍先判保护, 再算输出):
 *      锁存故障(过温/急停) > 温度无效 > 过温跳闸 > 高于目标 > 正常 PI
 ******************************************************************************
 */
#include "Heater.h"
#include "Parameter.h"
#include "BMI088.h"
#include "math.h"

Heater_t Heater =
{
    /* 先给默认目标(Heater_Init 会再写一次): IMU_Cal 在任务启动早期就会读
     * Heater.Target_C/Temp_C 判断"是否已经恒温", 不能等 Heater_Init 才有效 */
    .Target_C = 40.0f,
    .Fault    = HEATER_FAULT_NONE,
};

/*------------------------------ 可调参数 ----------------------------------
 * 目标温度 / 保护阈值 / PI 增益 / 无响应保护 全部在 Parameter.h 第 6 节
 * 「IMU 恒温加热」里, 本文件不再定义。
 *--------------------------------------------------------------------------*/

static uint32_t Heater_LastWindowMs = 0;
static uint8_t  Heater_WindowStarted = 0;

/* 无响应保护用的累计量 */
static float    Heater_HighTime_s = 0.0f;   /* 满功率连续加热时长(s) */
static float    Heater_T_Base     = 0.0f;   /* 开始满功率时的温度(℃) */

/*============================ 输出层(只用寄存器) ==========================*/
static void Heater_OutputOff(void)
{
    /* 顺序很重要: 先把 CCR 置 0, 让 PWM1 的比较输出立刻无效(CNT>=CCR),
     * 再停计数器 —— 否则中途停表时 OC1REF 可能仍为高, 加热管就一直是开的。 */
    TIM4->CCR1  = 0;
    TIM4->CR1  &= ~TIM_CR1_CEN;
    TIM4->CNT   = 0;
    Heater.Duty = 0.0f;
}

static void Heater_StartWindow(uint32_t Ccr)
{
    if (Ccr == 0u) { Heater_OutputOff(); return; }
    if (Ccr > HEATER_ARR) Ccr = HEATER_ARR;

    TIM4->CCR1 = Ccr;              /* 无预装载, 立即生效 */
    TIM4->CNT  = 0;
    TIM4->CR1 |= TIM_CR1_CEN;      /* 单脉冲: 到 ARR 自动停 */
}

/*================================= 初始化 ================================*/
void Heater_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* TIM4: 170MHz/(169+1) = 1MHz 计数, ARR=19999 -> 20ms 窗口, CH1 PWM1 */
    TIM4->CR1    = 0;                                   /* 停止, 向上计数 */
    TIM4->PSC    = 169u;
    TIM4->ARR    = HEATER_ARR;
    TIM4->CCR1   = 0;
    TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE); /* 关预装载: 写 CCR 立即生效 */
    TIM4->CCMR1 |= TIM_OCMODE_PWM1;                     /* PWM1: CNT<CCR 输出高 */
    TIM4->CCER  &= ~TIM_CCER_CC1P;                      /* 高电平有效 */
    TIM4->CCER  |= TIM_CCER_CC1E;
    TIM4->CR1   |= (TIM_CR1_OPM | TIM_CR1_ARPE);        /* 单脉冲模式 */
    TIM4->EGR    = TIM_EGR_UG;                          /* 装载 PSC/ARR */
    TIM4->SR     = 0;
    TIM4->CR1   &= ~TIM_CR1_CEN;

    Heater.Target_C       = HEATER_TARGET_C;
    Heater.Temp_C         = 0.0f;
    Heater.Duty           = 0.0f;
    Heater.Integral       = 0.0f;
    Heater.Window_Count   = 0;
    Heater.Fault_Count    = 0;
    Heater.Fault          = HEATER_FAULT_NONE;
    Heater_WindowStarted  = 0;
    Heater_LastWindowMs   = 0;
    Heater_HighTime_s     = 0.0f;
    Heater_T_Base         = 0.0f;

    Heater_OutputOff();
}

/*============================== 闭环控制 =================================*/
void Heater_Update(float Dt)
{
    uint32_t now = HAL_GetTick();
    float T, e, duty, i_new, prev_duty;

    /* 1) 锁存故障: 无条件先关 */
    if ((Heater.Fault == HEATER_FAULT_OVERTEMP) ||
        (Heater.Fault == HEATER_FAULT_NO_RESPONSE) ||
        (Heater.Fault == HEATER_FAULT_EMERGENCY))
    {
        Heater_OutputOff();
        return;
    }

    /* 2) 温度有效性: 传感器没就绪 / 超过 100ms 没更新 / 数值非法 -> 关 */
    if ((BMI088.Ready == 0u) ||
        (BMI088.Temp_Tick == 0u) ||
        ((uint32_t)(now - BMI088.Temp_Tick) > HEATER_TEMP_TIMEOUT_MS))
    {
        if (Heater.Fault != HEATER_FAULT_NO_SENSOR) Heater.Fault_Count++;
        Heater.Fault    = HEATER_FAULT_NO_SENSOR;
        Heater.Integral = 0.0f;
        Heater_OutputOff();
        return;
    }

    T = BMI088.Temperature;
    Heater.Temp_C = T;

    if (!isfinite(T) ||
        (T < HEATER_TEMP_MIN_VALID) || (T > HEATER_TEMP_MAX_VALID))
    {
        if (Heater.Fault != HEATER_FAULT_NO_SENSOR) Heater.Fault_Count++;
        Heater.Fault    = HEATER_FAULT_NO_SENSOR;
        Heater.Integral = 0.0f;
        Heater_OutputOff();
        return;
    }

    /* 3) 过温跳闸: 锁存, 必须人工清除 */
    if (T >= HEATER_TRIP_C)
    {
        Heater.Fault_Count++;
        Heater.Fault    = HEATER_FAULT_OVERTEMP;
        Heater.Integral = 0.0f;
        Heater_OutputOff();
        return;
    }

    Heater.Fault = HEATER_FAULT_NONE;

    /* 本拍时间步长(用于积分) */
    if (!(Dt > 0.0f))  Dt = (float)HEATER_TIM_WINDOW_MS * 0.001f;
    if (Dt > 0.5f)     Dt = 0.5f;

    /* 4) 已经高于目标: 不加热, 积分按负误差往回收(避免冷却后再加热冲过头) */
    if (T >= HEATER_SOFT_STOP_C)
    {
        Heater.Integral += HEATER_KI * (HEATER_TARGET_C - T) * Dt;
        if (Heater.Integral < 0.0f) Heater.Integral = 0.0f;
        Heater_OutputOff();
        return;
    }

    /* 5) 窗口节拍: 每 20ms 只发一个单脉冲(调用再快也不会把占空比放大) */
    if ((Heater_WindowStarted != 0u) &&
        ((uint32_t)(now - Heater_LastWindowMs) < HEATER_TIM_WINDOW_MS))
    {
        return;
    }
    Heater_WindowStarted = 1;
    Heater_LastWindowMs  = now;

    /* 6) 前馈 + PI(抗积分饱和) */
    e = HEATER_TARGET_C - T;

    i_new = Heater.Integral + HEATER_KI * e * Dt;
    if (i_new < 0.0f)          i_new = 0.0f;
    if (i_new > HEATER_DUTY_MAX) i_new = HEATER_DUTY_MAX;

    /* 前馈只在"低于目标"时给, 避免温度已经到点了还被前馈顶着继续加热 */
    duty = ((e > 0.0f) ? HEATER_FF_DUTY : 0.0f) + HEATER_KP * e + i_new;

    if (duty > HEATER_DUTY_MAX)
    {
        duty = HEATER_DUTY_MAX;
        if (e > 0.0f) i_new = Heater.Integral;   /* 饱和时不再往上积分 */
    }
    if (duty < 0.0f) duty = 0.0f;

    Heater.Integral = i_new;

    /* 7) 占空比斜率限制(避免突然满功率) */
    prev_duty = Heater.Duty;
    if (duty > prev_duty + HEATER_DUTY_SLEW) duty = prev_duty + HEATER_DUTY_SLEW;
    if (duty < prev_duty - HEATER_DUTY_SLEW) duty = prev_duty - HEATER_DUTY_SLEW;
    if (duty < 0.0f) duty = 0.0f;

    /* 8) 高于目标 + 死区: 一律不加热(放在斜率限制之后, 否则会被"每窗口最多降
     *    10%"的斜率限制又拉回去继续加热) */
    if (T >= (HEATER_TARGET_C + HEATER_DEADBAND_C)) duty = 0.0f;

    /* 9) 无响应保护: 接近满功率连续加热很久, 温度却几乎不涨
     * -> 温度传感器卡死 / 加热丝断 / 5V 没电。锁存, 不再加热。 */
    if (T >= (HEATER_TARGET_C - 1.0f))
    {
        Heater_HighTime_s = 0.0f;
    }
    else if (duty >= HEATER_NO_RESPONSE_DUTY)
    {
        if (Heater_HighTime_s <= 0.0f) Heater_T_Base = T;
        Heater_HighTime_s += Dt;
        if ((Heater_HighTime_s >= HEATER_NO_RESPONSE_S) &&
            ((T - Heater_T_Base) < HEATER_NO_RESPONSE_DT))
        {
            Heater.Fault_Count++;
            Heater.Fault      = HEATER_FAULT_NO_RESPONSE;
            Heater.Integral   = 0.0f;
            Heater_HighTime_s = 0.0f;
            Heater_OutputOff();
            return;
        }
    }
    else
    {
        Heater_HighTime_s = 0.0f;
    }

    if (duty <= 0.0f)
    {
        Heater_OutputOff();
        return;
    }

    /* 10) 输出一个窗口: 宽度 = duty% × 20ms */
    Heater_StartWindow((uint32_t)(duty * (float)(HEATER_ARR + 1u) / 100.0f));
    Heater.Duty = duty;
    Heater.Window_Count++;
}

/*============================ 紧急关断/清故障 ============================*/
/* 可从异常处理/中断里调用: 只有寄存器写和静态变量赋值, 不调用 HAL、不阻塞 */
void Heater_EmergencyOff(void)
{
    TIM4->CCR1  = 0;
    TIM4->CR1  &= ~TIM_CR1_CEN;
    TIM4->CNT   = 0;

    Heater.Duty     = 0.0f;
    Heater.Integral = 0.0f;
    if (Heater.Fault != HEATER_FAULT_OVERTEMP)
        Heater.Fault = HEATER_FAULT_EMERGENCY;
}

void Heater_ClearFault(void)
{
    if (Heater.Fault == HEATER_FAULT_EMERGENCY) return;   /* 系统错误不自动恢复 */

    if (Heater.Fault == HEATER_FAULT_OVERTEMP)
    {
        /* 还热着不许清: 必须等温度回到目标以下 */
        if ((BMI088.Ready == 0u) || (BMI088.Temperature >= HEATER_TARGET_C)) return;
    }

    Heater.Fault       = HEATER_FAULT_NONE;
    Heater.Integral    = 0.0f;
    Heater_HighTime_s  = 0.0f;
    Heater_OutputOff();
}
