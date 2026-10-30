#ifndef __Buzzer_Task__
#define __Buzzer_Task__

#include "main.h"

//蜂鸣器音效任务间隔，10ms。建议不要高于30。
#define BUZZER_TASK_CONTROL_TIME 10

/*===| 预设音效枚举定义 |===*/
typedef enum
{
	Buzzer_SoundEffect_OFF = 0,
    Buzzer_SoundEffect_SystemStart,
    Buzzer_SoundEffect_SuperCap_ON,
    Buzzer_SoundEffect_SuperCap_OFF,
    Buzzer_SoundEffect_Aim_ON,
    Buzzer_SoundEffect_Aim_OFF,
    Buzzer_SoundEffect_Speed1,
    Buzzer_SoundEffect_Speed2,
    Buzzer_SoundEffect_Speed3,
    Buzzer_SoundEffect_Speed4,
    Buzzer_SoundEffect_Speed5,
    Buzzer_SoundEffect_Shoot1,
    Buzzer_SoundEffect_Shoot2,
    Buzzer_SoundEffect_Shoot3,
    Buzzer_SoundEffect_Shoot4,
    Buzzer_SoundEffect_Shoot5,
    Buzzer_SoundEffect_Shoot6,
    Buzzer_SoundEffect_Shoot7,
    Buzzer_SoundEffect_Shoot8,
	  Buzzer_SoundEffect_Nav_On,
	  Buzzer_SoundEffect_Nav_Off,
		Buzzer_SoundEffect_Search_On,
		Buzzer_SoundEffect_Search_Off,
		Buzzer_SoundEffect_Fire_Auto_On,
		Buzzer_SoundEffect_Fire_Auto_Off,
		Buzzer_SoundEffect_Gimbal_Position_Change,
	Buzzer_SoundEffect_Gyro_Cal_Done,

    /* ===| IMU 交互标定提示音(六个面 + 两个参考姿态 + 完成, 各自音型不同) |=== */
    Buzzer_SoundEffect_IMU_Cal_Start,   /* 标定开始                            */
    Buzzer_SoundEffect_IMU_Cal_Pose1,   /* 第1个面采集完成                     */
    Buzzer_SoundEffect_IMU_Cal_Pose2,   /* 第2个面(低音2声)                    */
    Buzzer_SoundEffect_IMU_Cal_Pose3,   /* 第3个面(低音3声)                    */
    Buzzer_SoundEffect_IMU_Cal_Pose4,   /* 第4个面(高音1声)                    */
    Buzzer_SoundEffect_IMU_Cal_Pose5,   /* 第5个面(高音2声)                    */
    Buzzer_SoundEffect_IMU_Cal_Pose6,   /* 第6个面(高音3声)                    */
    Buzzer_SoundEffect_IMU_Cal_RefUp,   /* 重力方向(正面朝上)锁定              */
    Buzzer_SoundEffect_IMU_Cal_RefFwd,  /* 机头(前向)方向锁定                  */
    Buzzer_SoundEffect_IMU_Cal_Done,    /* 标定完成并已写入 Flash              */
} Buzzer_SoundEffect_EnumTypedef;

/*===| 蜂鸣器音调枚举定义 |===*/
typedef enum
{
	P = 0,    
    L1, L2, L3, L4, L5, L6, L7,
    M1, M2, M3, M4, M5, M6, M7,
    H1, H2, H3, H4, H5, H6, H7,
} Buzzer_Tone_EnumTypedef;


void Buzzer_Init(void);

void Buzzer_Set_SoundEffect(Buzzer_SoundEffect_EnumTypedef SoundEffect);
/* 排队播放: 若当前有音效正在播(如开机音效), 会等它播完后补播, 不会丢事件 */
void Buzzer_Queue_SoundEffect(Buzzer_SoundEffect_EnumTypedef SoundEffect);

/* 1 = 还有音效在播或在排队(排队区深度只有 1, 连续排两个会覆盖) */
uint8_t Buzzer_Is_Busy(void);
/* 等音效播完, 最多 TimeoutMs; 返回 1 = 已空闲 */
uint8_t Buzzer_Wait_Idle(uint32_t TimeoutMs);

void Buzzer_Set_Tone(Buzzer_Tone_EnumTypedef Tone);

#endif


