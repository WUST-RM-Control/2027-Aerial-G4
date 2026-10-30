#include "Sensor.h"

#include "Parameter.h"
#include "BMP581.h"
#include "BMI088.h"
#include "Heater.h"
#include "Buzzer.h"
#include "Control.h"


Sensor_StructTypedef Sensor;

uint32_t Sensor_Task_Ticker;
float Sensor_Task_Dt;
void Sensor_Task(void *argument)
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); //ADC
    BMP581_Init();    // V3: 气压计改成 BMP581(SPI1 + PB1 数据就绪中断)
    Heater_Init();    // IMU 恒温加热: TIM4_CH1 单脉冲窗口, 默认关闭输出

    DWT_GetDeltaT(&Sensor_Task_Ticker);

    for(;;)
    {
        Sensor_Task_Dt = DWT_GetDeltaT(&Sensor_Task_Ticker);

        //获取传感器数据(只有 INT 引脚来了新样本才真正读 SPI, 见 BMP581.c)
        BMP581_Read_Data();    // 气压Pa/温度/海拔/相对高度

        //电池电压采集(单次转换: 先等 EOC 再读, 否则读到的是上一次的结果)
        {
            static uint8_t  LowVoltAlarm = 0;
            static uint32_t AlarmStartMs = 0;

            if (AlarmStartMs == 0u) AlarmStartMs = HAL_GetTick();

            HAL_ADC_Start(&hadc2);
            if (HAL_ADC_PollForConversion(&hadc2, 5u) == HAL_OK)
            {
                float bat_v = HAL_ADC_GetValue(&hadc2) * 3.3f / 4096.0f * 3.0f;
                Filter_LowPass(bat_v, &Sensor.BatV, SENSOR_TAU_VBAT, Sensor_Task_Dt);
            }
            HAL_ADC_Stop(&hadc2);

            /* 低电压告警: 只在"刚进入低电压"时响一次, 回到 7.3V 以上才复位。
             * 原来每 20ms 重设一次音效, 会让蜂鸣器卡在同一个音上一直响。
             * 上电前 3s 不报警: 滤波从 0 爬升到真实电压的过程会经过 5~7V。 */
            if (LowVoltAlarm)
            {
                if ((Sensor.BatV > 7.3f) || (Sensor.BatV < 5.0f)) LowVoltAlarm = 0;
            }
            else if (((uint32_t)(HAL_GetTick() - AlarmStartMs) > 3000u) &&
                     (Sensor.BatV <= 7.0f) && (Sensor.BatV >= 5.0f))
            {
                LowVoltAlarm = 1;
                Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_ON);
            }
        }

        //气压计数据
        Bar_Update(&Sensor.Bar_Height_mm, &Sensor.Bar_Height_Speed_mm, BMP581.Relative_Height_mm, Sensor_Task_Dt);

        /* IMU 恒温加热(40℃闭环): 每 20ms 一拍, 内部自带全部保护 */
        Heater_Update(Sensor_Task_Dt);

        /* V3 控制板上没有光流(PMW3901)和 TOF(VL53L1): 高度只用气压计,
         * 水平速度没有观测量, 保持 0(定点模式会退化成"定点姿态保持")。 */
        Sensor.Height_mm       = Sensor.Bar_Height_mm;
        Sensor.Height_Speed_mm = Sensor.Bar_Height_Speed_mm;
        Sensor.Vel_X_mm        = 0.0f;
        Sensor.Vel_Y_mm        = 0.0f;

        //低电压告警在电池电压采集处处理(边沿触发), 这里不再重复设置

        osDelay(20);

    }
}

/*============================================================================
 *  传感器 INT 引脚统一回调(PB1 / PA9 / PA10 都走这里)
 *    只置标志/计数, 不在这里做 SPI, 由对应任务去读
 *===========================================================================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case BPS_INT_Pin:       BMP581_INT_Callback();      break;  /* 气压计数据就绪 */
    case IMU_INT_ACC_Pin:   BMI088_INT_Acc_Callback();  break;  /* 加速度计数据就绪 */
    case IMU_INT_GYRO_Pin:  BMI088_INT_Gyro_Callback(); break;  /* 陀螺仪数据就绪 */
    default: break;
    }
}


void Bar_Update(float *Bar_Height_Out, float *Bar_Height_Speed_Out, float Bar_Height, float Dt)
{
    static float Out_Height_Last;
    float Average_Thrust = (Control.Duty[0] + Control.Duty[1] + Control.Duty[2] + Control.Duty[3]) / 4.0f;

    Filter_LowPass((Bar_Height - SENSOR_FAN_K * Average_Thrust), Bar_Height_Out, SENSOR_TAU_BAR, Sensor_Task_Dt);
    Filter_LowPass(((*Bar_Height_Out - Out_Height_Last) / Dt), Bar_Height_Speed_Out, SENSOR_TAU_BAR_SPEED, Dt);
    Out_Height_Last = *Bar_Height_Out;
}
