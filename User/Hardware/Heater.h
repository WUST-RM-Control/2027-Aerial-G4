#ifndef __HEATER_H__
#define __HEATER_H__

#include "main.h"

/*==============================================================================
 *  IMU 恒温加热(PA11 = TIM4_CH1 -> 120Ω -> AO3400 栅极, 5V/120Ω 加热电阻)
 *
 *  控制: 目标 40℃, PI 闭环(带前馈), 每 20ms 一个加热窗口。
 *  输出: TIM4 单脉冲(OPM)模式 —— 每次软件触发只发一个宽度 = 占空比×20ms 的
 *        脉冲, 到 20ms 硬件自动停。软件跑飞/任务卡死时最多再加热 20ms,
 *        不会出现"PWM 一直开着持续加热"。
 *
 *  保护(任何一条成立都立刻把 PWM 拉低):
 *    1. 温度读不到/过期(BMI088 未就绪、>100ms 没更新、数值非法) -> 关
 *    2. 过温(T >= 46℃) -> 关并锁存, 需 Heater_ClearFault() 且温度降下来才能恢复
 *    3. 温度高于目标(>=41℃) -> 关, 积分按负误差回收
 *    4. 高于目标 +0.3℃ 死区 -> 不给输出(防止在目标点附近反复加热)
 *    5. 满功率加热 180s 温度几乎不涨 -> 判传感器卡死/加热丝断, 关并锁存
 *    6. 任何 Error_Handler / HardFault / NMI / MemManage / BusFault / UsageFault
 *       的第一条语句就是 Heater_EmergencyOff()
 *    7. 上电时 main.c 在 MX_GPIO_Init 之前就把 PA11 拉低(栅极另有 10k 下拉)
 *============================================================================*/
typedef enum
{
    HEATER_FAULT_NONE = 0,      /* 正常                                  */
    HEATER_FAULT_NO_SENSOR,     /* 温度无效/过期(可自动恢复)              */
    HEATER_FAULT_NO_RESPONSE,   /* 满功率加热很久温度不升(传感器卡死/加热丝断), 锁存 */
    HEATER_FAULT_OVERTEMP,      /* 过温, 锁存                             */
    HEATER_FAULT_EMERGENCY,     /* 系统错误/异常, 锁存                    */
} Heater_Fault_e;

typedef struct
{
    float Target_C;         /* 目标温度(℃)                   */
    float Temp_C;           /* 最近一次用于控制的温度(℃)      */
    float Duty;             /* 当前输出占空比(%)              */
    float Integral;         /* PI 积分项(%)                   */

    uint32_t Window_Count;  /* 已发出的加热窗口数             */
    uint32_t Fault_Count;   /* 进入故障的次数(调试用)         */

    Heater_Fault_e Fault;   /* 当前状态                       */
} Heater_t;

extern Heater_t Heater;

/* 初始化 TIM4 为 1MHz 计数、20ms 单脉冲窗口; 输出默认关闭 */
void Heater_Init(void);
/* 闭环控制: 建议每 ~20ms 调用一次, Dt = 实际间隔(s) */
void Heater_Update(float Dt);
/* 立即关断(可在中断/异常处理里调用, 只用寄存器写) */
void Heater_EmergencyOff(void);
/* 清除锁存故障(过温时要求当前温度已低于目标) */
void Heater_ClearFault(void);

#endif
