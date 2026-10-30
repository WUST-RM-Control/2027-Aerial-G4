#include "Buzzer.h"

#define TIMClock_Fequency 170000000
#define Buzzer_TIM &htim3
#define Buzzer_TIM_CHANNAL TIM_CHANNEL_2

Buzzer_SoundEffect_EnumTypedef Buzzer_SoundEffect;
/* 排队区(深度 1): 正在播的音效结尾会把自己的全局变量清 0, 所以直接写
 * 请求会有被丢掉的风险; 需要“一定要听到”的提示音(如陀螺标定完成)走这里 */
Buzzer_SoundEffect_EnumTypedef Buzzer_SoundEffect_Queue;

void Buzzer_Init(void)
{
    /*===| 启动蜂鸣器PWM |===*/
    HAL_TIM_Base_Start(Buzzer_TIM);
    HAL_TIM_PWM_Start(Buzzer_TIM, Buzzer_TIM_CHANNAL);
}

void Buzzer_Start(void)
{
    HAL_TIM_PWM_Start(Buzzer_TIM, Buzzer_TIM_CHANNAL);
}

void Buzzer_Stop(void)
{
    HAL_TIM_PWM_Stop(Buzzer_TIM, Buzzer_TIM_CHANNAL);
}

void Buzzer_Task(void const *argument)
{
    Buzzer_Init();
    
	for (;;)
	{
        /*===| 当前没有音效在播时, 把排队的请求提上来补播 |===*/
        /* (正在播的音效结尾会清 Buzzer_SoundEffect, 所以只能在这里播) */
        if ((Buzzer_SoundEffect == Buzzer_SoundEffect_OFF) &&
            (Buzzer_SoundEffect_Queue != Buzzer_SoundEffect_OFF))
        {
            Buzzer_SoundEffect = Buzzer_SoundEffect_Queue;
            Buzzer_SoundEffect_Queue = Buzzer_SoundEffect_OFF;
        }

        /*===| 根据需要播放的声音来分别播放音符 |===*/
        switch((uint8_t)Buzzer_SoundEffect)
        {
            case Buzzer_SoundEffect_OFF:
            {
                Buzzer_Set_Tone(P);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                Buzzer_Stop();
                break;
            }     
            case Buzzer_SoundEffect_SystemStart:
            {
                Buzzer_Start();
                Buzzer_Set_Tone(M1); osDelay(200); Buzzer_Set_Tone(P); osDelay(50); 
                Buzzer_Set_Tone(M2); osDelay(200); Buzzer_Set_Tone(P); osDelay(50);
                Buzzer_Set_Tone(M3); osDelay(200); Buzzer_Set_Tone(P); osDelay(50);
                Buzzer_Set_Tone(M2); osDelay(200); Buzzer_Set_Tone(P); osDelay(50);
                Buzzer_Set_Tone(M5); osDelay(200); Buzzer_Set_Tone(P); osDelay(50);
                Buzzer_Set_Tone(M7); osDelay(200); Buzzer_Set_Tone(P); osDelay(50);
                Buzzer_Set_Tone(M6); osDelay(500);
                Buzzer_Set_Tone(M5); osDelay(250);
                Buzzer_Set_Tone(M6); osDelay(750);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }    
            case Buzzer_SoundEffect_SuperCap_ON:
            {
                Buzzer_Start();
                Buzzer_Set_Tone(M1); osDelay(200);
                Buzzer_Set_Tone(M3); osDelay(200);
                Buzzer_Set_Tone(M5); osDelay(200);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }  
            case Buzzer_SoundEffect_SuperCap_OFF:
            {
                Buzzer_Start();
                Buzzer_Set_Tone(M5); osDelay(200);
                Buzzer_Set_Tone(M3); osDelay(200);
                Buzzer_Set_Tone(M1); osDelay(200);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }   
            case Buzzer_SoundEffect_Aim_ON:
            {
                Buzzer_Start();
                Buzzer_Set_Tone(M7); osDelay(50); Buzzer_Set_Tone(P); osDelay(150);
                Buzzer_Set_Tone(M7); osDelay(50); Buzzer_Set_Tone(P); osDelay(150);
                Buzzer_Set_Tone(M7); osDelay(50); Buzzer_Set_Tone(P); osDelay(150);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }  
            case Buzzer_SoundEffect_Aim_OFF:
            {
                Buzzer_Start();
                Buzzer_Set_Tone(M7); osDelay(200);
                Buzzer_Set_Tone(M5); osDelay(200);
                Buzzer_Set_Tone(M3); osDelay(200);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }  
            case Buzzer_SoundEffect_Shoot1:
            {
                Buzzer_Start();
                for(uint8_t i = 0; i < 1; i++)
                {
                    Buzzer_Set_Tone(M5); osDelay(100); Buzzer_Set_Tone(P); osDelay(100); 
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }     
            case Buzzer_SoundEffect_Shoot2:
            {
                Buzzer_Start();
                for(uint8_t i = 0; i < 2; i++)
                {
                    Buzzer_Set_Tone(M5); osDelay(100); Buzzer_Set_Tone(P); osDelay(100); 
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }   
            case Buzzer_SoundEffect_Shoot3:
            {
                Buzzer_Start();
                for(uint8_t i = 0; i < 3; i++)
                {
                    Buzzer_Set_Tone(M5); osDelay(100); Buzzer_Set_Tone(P); osDelay(100); 
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            } 
            case Buzzer_SoundEffect_Shoot4:
            {
                Buzzer_Start();
                for(uint8_t i = 0; i < 4; i++)
                {
                    Buzzer_Set_Tone(M5); osDelay(100); Buzzer_Set_Tone(P); osDelay(100); 
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            } 
            case Buzzer_SoundEffect_Shoot5:
            {
                Buzzer_Start();
                for(uint8_t i = 0; i < 5; i++)
                {
                    Buzzer_Set_Tone(M5); osDelay(100); Buzzer_Set_Tone(P); osDelay(100); 
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }   
            case Buzzer_SoundEffect_Shoot6:
            {
                Buzzer_Start();
                for(uint8_t i = 0; i < 6; i++)
                {
                    Buzzer_Set_Tone(M5); osDelay(100); Buzzer_Set_Tone(P); osDelay(100); 
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }   
            case Buzzer_SoundEffect_Shoot7:
            {
                Buzzer_Start();
                for(uint8_t i = 0; i < 7; i++)
                {
                    Buzzer_Set_Tone(M5); osDelay(100); Buzzer_Set_Tone(P); osDelay(100); 
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }      
            case Buzzer_SoundEffect_Shoot8:
            {
                Buzzer_Start();
                for(uint8_t i = 0; i < 8; i++)
                {
                    Buzzer_Set_Tone(M5); osDelay(100); Buzzer_Set_Tone(P); osDelay(100); 
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }

            case Buzzer_SoundEffect_Nav_On:
            {
							  Buzzer_Start();
							  for(uint8_t i = 0; i < 3; i++)
							  {
									Buzzer_Set_Tone(H4);osDelay(75);Buzzer_Set_Tone(P); osDelay(75);
								}
						    Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
								break;
						}	
            case Buzzer_SoundEffect_Nav_Off:
						{
							Buzzer_Start();
							for(uint8_t i = 0; i < 3; i++)
							{
							    Buzzer_Set_Tone(L4);osDelay(125);Buzzer_Set_Tone(P); osDelay(125);
							}
						  Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
							break;
						  
						}
						
						case Buzzer_SoundEffect_Search_On:
						{
							Buzzer_Start();
							for(uint8_t i = 0; i < 2; i++)
							{
							    Buzzer_Set_Tone(H1);osDelay(75);Buzzer_Set_Tone(P); osDelay(75);
							}
						  Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
							break;
						  
						}
						
						case Buzzer_SoundEffect_Search_Off:
						{
							Buzzer_Start();
							for(uint8_t i = 0; i < 2; i++)
							{
							    Buzzer_Set_Tone(L1);osDelay(125);Buzzer_Set_Tone(P); osDelay(125);
							}
						  Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
							break;
						  
						}
						
						case Buzzer_SoundEffect_Fire_Auto_Off:
						{
							Buzzer_Start();
							for(uint8_t i = 0; i < 4; i++)
							{
							    Buzzer_Set_Tone(L1);osDelay(75);Buzzer_Set_Tone(P); osDelay(75);
							}
						  Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
							break;
						  
						}		

            case Buzzer_SoundEffect_Fire_Auto_On:
						{
							Buzzer_Start();
							for(uint8_t i = 0; i < 4; i++)
							{
							    Buzzer_Set_Tone(H1);osDelay(75);Buzzer_Set_Tone(P); osDelay(75);
							}
						  Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
							break;
						  
						}	

            case Buzzer_SoundEffect_Gyro_Cal_Done:
            {
                /*===| 陀螺零漂标定完成: 三连升音(约 0.6s), 区别于其它提示音 |===*/
                Buzzer_Start();
                Buzzer_Set_Tone(M3); osDelay(90);  Buzzer_Set_Tone(P); osDelay(60);
                Buzzer_Set_Tone(M5); osDelay(90);  Buzzer_Set_Tone(P); osDelay(60);
                Buzzer_Set_Tone(M7); osDelay(260); Buzzer_Set_Tone(P); osDelay(60);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }
            case Buzzer_SoundEffect_Gimbal_Position_Change:
						{
							Buzzer_Start();
							for(uint8_t i = 0; i < 2; i++)
							{
							    Buzzer_Set_Tone(M4);osDelay(75);Buzzer_Set_Tone(P); osDelay(75);
							}
						  Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
							break;
						}	

            /* ================= IMU 交互标定提示音 =================
             * 六个面: 用"音高 + 响铃次数"区分
             *   第 1/2/3 个面 = 低音响 1/2/3 声
             *   第 4/5/6 个面 = 高音响 1/2/3 声
             * 后两个参考姿态用不同音型(长音 / 下滑音), 不会和"数铃声"混。 */
            case Buzzer_SoundEffect_IMU_Cal_Start:
            {
                /* 开始标定: 两短一长(中音上行) */
                Buzzer_Start();
                Buzzer_Set_Tone(M2); osDelay(120); Buzzer_Set_Tone(P); osDelay(100);
                Buzzer_Set_Tone(M2); osDelay(120); Buzzer_Set_Tone(P); osDelay(100);
                Buzzer_Set_Tone(M6); osDelay(300); Buzzer_Set_Tone(P);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }
            case Buzzer_SoundEffect_IMU_Cal_Pose1:
            case Buzzer_SoundEffect_IMU_Cal_Pose2:
            case Buzzer_SoundEffect_IMU_Cal_Pose3:
            case Buzzer_SoundEffect_IMU_Cal_Pose4:
            case Buzzer_SoundEffect_IMU_Cal_Pose5:
            case Buzzer_SoundEffect_IMU_Cal_Pose6:
            {
                uint8_t idx  = (uint8_t)Buzzer_SoundEffect - (uint8_t)Buzzer_SoundEffect_IMU_Cal_Pose1; /* 0~5 */
                uint8_t cnt  = (uint8_t)((idx % 3u) + 1u);                       /* 响 1~3 声      */
                Buzzer_Tone_EnumTypedef tone = (idx < 3u) ? L5 : H4;             /* 前3面低音/后3面高音 */

                Buzzer_Start();
                for (uint8_t i = 0; i < cnt; i++)
                {
                    Buzzer_Set_Tone(tone); osDelay(120);
                    Buzzer_Set_Tone(P);    osDelay(120);
                }
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }
            case Buzzer_SoundEffect_IMU_Cal_RefUp:
            {
                /* 重力方向(正面朝上)锁定: 一个长中音 */
                Buzzer_Start();
                Buzzer_Set_Tone(M5); osDelay(500); Buzzer_Set_Tone(P);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }
            case Buzzer_SoundEffect_IMU_Cal_RefFwd:
            {
                /* 机头(前向)锁定: 两短下滑高音 */
                Buzzer_Start();
                Buzzer_Set_Tone(H5); osDelay(110); Buzzer_Set_Tone(P); osDelay(60);
                Buzzer_Set_Tone(H2); osDelay(110); Buzzer_Set_Tone(P);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }
            case Buzzer_SoundEffect_IMU_Cal_Done:
            {
                /* 全部完成且已写入 Flash: 三音上行 + 收尾长音 */
                Buzzer_Start();
                Buzzer_Set_Tone(M5); osDelay(110); Buzzer_Set_Tone(P); osDelay(50);
                Buzzer_Set_Tone(M7); osDelay(110); Buzzer_Set_Tone(P); osDelay(50);
                Buzzer_Set_Tone(H5); osDelay(280); Buzzer_Set_Tone(P);
                Buzzer_SoundEffect = Buzzer_SoundEffect_OFF;
                break;
            }						
        }
        
		osDelay(BUZZER_TASK_CONTROL_TIME);
	}
}

/**
 * @brief 设置蜂鸣器音效
 */
void Buzzer_Set_SoundEffect(Buzzer_SoundEffect_EnumTypedef SoundEffect)
{
    Buzzer_SoundEffect = SoundEffect;
}

/**
 * @brief 排队播放音效(正在播的播完后补播, 不会被覆盖丢弃)
 */
void Buzzer_Queue_SoundEffect(Buzzer_SoundEffect_EnumTypedef SoundEffect)
{
    Buzzer_SoundEffect_Queue = SoundEffect;
}

/**
 * @brief 是否还有音效在播/在排队
 * @note  排队区深度只有 1: 连续排两个时, 后一个会覆盖前一个。
 *        需要保证两段声音都被听到时, 先用 Buzzer_Wait_Idle() 等一下。
 */
uint8_t Buzzer_Is_Busy(void)
{
    return ((Buzzer_SoundEffect != Buzzer_SoundEffect_OFF) ||
            (Buzzer_SoundEffect_Queue != Buzzer_SoundEffect_OFF)) ? 1u : 0u;
}

/**
 * @brief 等音效播完(最多 TimeoutMs)
 * @return 1 = 已空闲; 0 = 超时仍忙
 */
uint8_t Buzzer_Wait_Idle(uint32_t TimeoutMs)
{
    uint32_t t0 = HAL_GetTick();

    while (Buzzer_Is_Busy())
    {
        if ((uint32_t)(HAL_GetTick() - t0) >= TimeoutMs) return 0u;
        osDelay(10);
    }
    return 1u;
}

/**
 * @brief 设置蜂鸣器音调
 */
void Buzzer_Set_Tone(Buzzer_Tone_EnumTypedef Tone)
{
    uint16_t PSC;
    uint32_t ARL;
    
    switch((uint8_t)Tone)
    {
        case P:  PSC = 50;    ARL = 100;  break;
        case L1: PSC = 50;    ARL = TIMClock_Fequency / PSC / 262;  break;
        case L2: PSC = 50;    ARL = TIMClock_Fequency / PSC / 294;  break;
        case L3: PSC = 50;    ARL = TIMClock_Fequency / PSC / 330;  break;
        case L4: PSC = 50;    ARL = TIMClock_Fequency / PSC / 349;  break;
        case L5: PSC = 50;    ARL = TIMClock_Fequency / PSC / 392;  break;
        case L6: PSC = 50;    ARL = TIMClock_Fequency / PSC / 440;  break;
        case L7: PSC = 50;    ARL = TIMClock_Fequency / PSC / 494;  break;
        case M1: PSC = 50;    ARL = TIMClock_Fequency / PSC / 523;  break;
        case M2: PSC = 50;    ARL = TIMClock_Fequency / PSC / 587;  break;
        case M3: PSC = 50;    ARL = TIMClock_Fequency / PSC / 659;  break;
        case M4: PSC = 50;    ARL = TIMClock_Fequency / PSC / 698;  break;
        case M5: PSC = 50;    ARL = TIMClock_Fequency / PSC / 784;  break;
        case M6: PSC = 50;    ARL = TIMClock_Fequency / PSC / 880;  break;
        case M7: PSC = 50;    ARL = TIMClock_Fequency / PSC / 988;  break;
        case H1: PSC = 50;    ARL = TIMClock_Fequency / PSC / 1046; break;
        case H2: PSC = 50;    ARL = TIMClock_Fequency / PSC / 1175; break;
        case H3: PSC = 50;    ARL = TIMClock_Fequency / PSC / 1318; break;
        case H4: PSC = 50;    ARL = TIMClock_Fequency / PSC / 1397; break;
        case H5: PSC = 50;    ARL = TIMClock_Fequency / PSC / 1568; break;
        case H6: PSC = 50;    ARL = TIMClock_Fequency / PSC / 1760; break;
        case H7: PSC = 50;    ARL = TIMClock_Fequency / PSC / 1976; break;
    }
    __HAL_TIM_SET_PRESCALER(Buzzer_TIM, PSC - 1);
    __HAL_TIM_SET_AUTORELOAD(Buzzer_TIM, ARL - 1);
    __HAL_TIM_SetCompare(Buzzer_TIM, Buzzer_TIM_CHANNAL, ARL / 2);
}
