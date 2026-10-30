#include "Communicate.h"
#include "Sensor.h"
#include "INS.h"
#include "Control.h"

void Communicate_Task(void *argument)
{
    osDelay(50);

    /* TIM7 独立节拍(默认 1kHz, 见 Vofa.h 的 VOFA_SEND_HZ)。
     * 与飞行主循环(TIM6)完全分开, 两边频率可以各自调、互不影响。
     * 发送口 = USART2(调试口), 4 Mbaud + DMA。 */
    Vofa_Tick_Init();

    for(;;)
    {
        Vofa_Wait_Tick();   /* 阻塞到下一个 TIM7 节拍(不再用 osDelay(1)) */
    //    Vofa_JF_Add_Data(0, Sensor.BatV);

       Vofa_JF_Add_Data(0, Sensor.Bar_Height_mm);
       Vofa_JF_Add_Data(2, Sensor.Height_mm);
       Vofa_JF_Add_Data(3, Sensor.Height_Speed_mm);
        
    //    Vofa_JF_Add_Data(0, Control.Duty[0]);
    //    Vofa_JF_Add_Data(1, Control.Duty[1]);
    //    Vofa_JF_Add_Data(2, Control.Duty[2]);
    //    Vofa_JF_Add_Data(3, Control.Duty[3]);
    //    Vofa_JF_Add_Data(4, Control.T_Height);
    //    Vofa_JF_Add_Data(5, Control.T_Pitch);
    //    Vofa_JF_Add_Data(6, Control.T_Roll);
    //    Vofa_JF_Add_Data(7, Control.T_Yaw);
        
        
        // Vofa_JF_Add_Data(0, Control.PID_P_Height.Target);
        // Vofa_JF_Add_Data(1, Control.PID_P_Height.Feedback);
        // Vofa_JF_Add_Data(2, Control.PID_S_Height.Target);
        // Vofa_JF_Add_Data(3, Control.PID_S_Height.Feedback);
        // Vofa_JF_Add_Data(4, Control.PID_S_Height.Output);

    //    Vofa_JF_Add_Data(0, INS.YawTotalAngle);
    //    Vofa_JF_Add_Data(1, INS.Roll);
    //    Vofa_JF_Add_Data(2, INS.Pitch);
    //    Vofa_JF_Add_Data(3, INS.Yaw_Speed);
    //    Vofa_JF_Add_Data(4, INS.Roll_Speed);
    //    Vofa_JF_Add_Data(5, INS.Pitch_Speed);
        

        Vofa_JF_Send();
    }
}

