#include "Communicate.h"
#include "Sensor.h"
#include "INS.h"
#include "Control.h"
#include "PMW3901.h"

void Communicate_Task(void *argument)
{
    osDelay(50);
    
    for(;;)
    {
//        Vofa_JF_Add_Data(0, Sensor.BatV);
//        Vofa_JF_Add_Data(1, Sensor.Flow_Pix_Vx);
//        Vofa_JF_Add_Data(2, Sensor.Flow_Pix_Vy);
//        Vofa_JF_Add_Data(3, Sensor.Bar_Height_mm);
//        Vofa_JF_Add_Data(4, Sensor.TOF_Height_mm);
//        Vofa_JF_Add_Data(5, INS.YawTotalAngle);
//        Vofa_JF_Add_Data(6, INS.Roll);
//        Vofa_JF_Add_Data(7, INS.Pitch);
//        Vofa_JF_Add_Data(8, Sensor.Height_mm);
//        Vofa_JF_Add_Data(9, Sensor.Height_Speed_mm);
//        Vofa_JF_Add_Data(10, Sensor.Vel_X_mm);
//        Vofa_JF_Add_Data(11, Sensor.Vel_Y_mm);
//        Vofa_JF_Add_Data(12, PMW3901.SQUAL);

        
        Vofa_JF_Add_Data(0, Control.Duty[0]);
        Vofa_JF_Add_Data(1, Control.Duty[1]);
        Vofa_JF_Add_Data(2, Control.Duty[2]);
        Vofa_JF_Add_Data(3, Control.Duty[3]);
        Vofa_JF_Add_Data(4, Control.T_Height);
        Vofa_JF_Add_Data(5, Control.T_Pitch);
        Vofa_JF_Add_Data(6, Control.T_Roll);
        Vofa_JF_Add_Data(7, Control.T_Yaw);
        
        /* ---- 高度融合诊断用(判断"速度偏置/漂移"来自哪一路): 取消下面注释、
         *      并注释掉上面的俯仰 PID 组即可 ----
         * 期望: Tof_Weight≈1, V_Ref≈0, Height_Speed≈0, Acc_Bias 稳定不变 */
//        Vofa_JF_Add_Data(0, Sensor.Height_mm);
//        Vofa_JF_Add_Data(1, Sensor.Height_Speed_mm);
//        Vofa_JF_Add_Data(2, Sensor.V_Ref_mm);
//        Vofa_JF_Add_Data(3, Sensor.Acc_Bias_mm_s2);
//        Vofa_JF_Add_Data(4, Sensor.Baro_F_mm);
//        Vofa_JF_Add_Data(5, Sensor.Tof_Weight);
//        Vofa_JF_Add_Data(6, INS.MotionAccel_n[2] * 1000.0f);   /* 加计竖直加速度 mm/s² */
        
//        Vofa_JF_Add_Data(0, Control.PID_P_Roll.Target);
//        Vofa_JF_Add_Data(1, Control.PID_P_Roll.Feedback);
//        Vofa_JF_Add_Data(2, Control.PID_S_Roll.Target);
//        Vofa_JF_Add_Data(3, Control.PID_S_Roll.Feedback);
//        Vofa_JF_Add_Data(4, Control.PID_S_Roll.Output);
//        Vofa_JF_Add_Data(5, Control.PID_P_Pitch.Target);
//        Vofa_JF_Add_Data(6, Control.PID_P_Pitch.Feedback);
//        Vofa_JF_Add_Data(7, Control.PID_S_Pitch.Target);
//        Vofa_JF_Add_Data(8, Control.PID_S_Pitch.Feedback);
//        Vofa_JF_Add_Data(9, Control.PID_S_Pitch.Output);
//                                         
//        Vofa_JF_Add_Data(6, Control.PID_S_Pitch.K_Output);
//        Vofa_JF_Add_Data(7, Control.PID_S_Pitch.I_Output);
//        Vofa_JF_Add_Data(8, Control.PID_S_Pitch.F_Output);
        
//        Vofa_JF_Add_Data(0, INS.YawTotalAngle);
//        Vofa_JF_Add_Data(1, INS.Roll);
//        Vofa_JF_Add_Data(2, INS.Pitch);
//        Vofa_JF_Add_Data(3, INS.Yaw_Speed);
//        Vofa_JF_Add_Data(4, INS.Roll_Speed);
//        Vofa_JF_Add_Data(5, INS.Pitch_Speed);

        Vofa_JF_Send();
        
        osDelay(10);
    }
}

