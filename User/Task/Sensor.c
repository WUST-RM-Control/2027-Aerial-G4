#include "Sensor.h"

#include "PMW3901.h"
#include "SPA06.h"
#include "VL53L1.h"
#include "Buzzer.h"
#include "INS.h"
#include "Control.h"

#include "math.h"   // fabsf


Sensor_StructTypedef Sensor;

#define SENSOR_TAU_VBAT          0.5f   //电池电压一阶低通时间常数(s)
#define SENSOR_VBAT_VALID_MIN    1.5f   /* 低于此视为无效(转换未完成/未接电池), 不参与滤波 */

uint32_t Sensor_Task_Ticker;
float Sensor_Task_Dt;
void Sensor_Task(void *argument)
{
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED); //ADC
    PMW3901_Init();   // SPI1 -> 模式3
    SPA06_Init();     // I2C3 由 main.c 的 MX_I2C3_Init() 初始化
    VL53L1_Init();    // 写默认配置+一次自检, 耗时较长
    
    DWT_GetDeltaT(&Sensor_Task_Ticker);
    
    for(;;)
    {     
        Sensor_Task_Dt = DWT_GetDeltaT(&Sensor_Task_Ticker);
        
        //获取传感器数据
        PMW3901_Read_Data();   // 光流原始计数
        SPA06_Read_Data();     // 气压Pa/温度/海拔
        VL53L1_Read_Data();   // 距离mm; 非阻塞, 只有拿到新样本才返回1
        
        //电池电压采集
        HAL_ADC_Start(&hadc2);
        Filter_LowPass((HAL_ADC_GetValue(&hadc2) * 3.3f/4096.0f * 3.0f), &Sensor.BatV, SENSOR_TAU_VBAT, Sensor_Task_Dt);
        
        //光流数据
        Sensor.Flow_Pix_Vx = -PMW3901.Delta_X / Sensor_Task_Dt;
        Sensor.Flow_Pix_Vy = PMW3901.Delta_Y / Sensor_Task_Dt;
        
        //气压计数据
        Bar_Update(&Sensor.Bar_Height_mm, &Sensor.Bar_Height_Speed_mm, SPA06.Relative_Height_mm, Sensor_Task_Dt);
        
        //TOF数据
        /* 倾斜补偿的 cos(倾角) 由四元数给出: 机体Z轴在导航系下的Z分量(=INS.zn[2]),
         * 大角度/倒扣时不会像欧拉角那样折叠, 也不会在中途失效 */
        TOF_Update(&Sensor.TOF_Height_mm, &Sensor.TOF_Height_Speed_mm, VL53L1.Distance_mm, INS.zn[2], Sensor_Task_Dt);
        
        //高度数据融合
//        Sensor.Height_mm       = Sensor.Bar_Height_mm;
//        Sensor.Height_Speed_mm = Sensor.Bar_Height_Speed_mm;
        Sensor.Height_mm       = Sensor.TOF_Height_mm;
        Sensor.Height_Speed_mm = Sensor.TOF_Height_Speed_mm;
        
        
        //光流数据处理
        Sensor_Flow_Update(&Sensor.Vel_X_mm, &Sensor.Vel_Y_mm,
                            Sensor.Flow_Pix_Vx, Sensor.Flow_Pix_Vy, PMW3901.SQUAL,
                            Sensor.TOF_Height_mm, -INS.Roll_Speed*Angle2Rad, INS.Pitch_Speed*Angle2Rad,
                            Sensor_Task_Dt);

        //低电压警告
        if(Sensor.BatV <= 7.0f && Sensor.BatV >= 5.0f) Buzzer_Set_SoundEffect(Buzzer_SoundEffect_Aim_ON);
        
        osDelay(20);
        
    }
}



#define SENSOR_TAU_Bar             0.3f   //气压高度一阶低通时间常数(s)
#define SENSOR_TAU_Bar_Speed       0.2f   //气压速度一阶低通时间常数(s)
#define SENSOR_Fan_K               13.44f   //高度随转速的补偿比例(mm/1%)
void Bar_Update(float *Bar_Height_Out, float *Bar_Height_Speed_Out, float Bar_Height, float Dt)
{
    static float Out_Height_Last;
    float Average_Thrust = (Control.Duty[0] + Control.Duty[1] + Control.Duty[2] + Control.Duty[3]) / 4.0f;
    
    Filter_LowPass((Bar_Height - SENSOR_Fan_K * Average_Thrust), Bar_Height_Out, SENSOR_TAU_Bar, Sensor_Task_Dt);
    Filter_LowPass(((*Bar_Height_Out - Out_Height_Last) / Dt), Bar_Height_Speed_Out, SENSOR_TAU_Bar_Speed, Dt);
    
    Out_Height_Last = *Bar_Height_Out;
}



/* ================= TOF 高度卡尔曼滤波参数(集中在这里调) =================
 * 状态 x = [高度(mm), 速度(mm/s)]^T, F = [1 Dt; 0 1], 量测 z = 倾角补偿后的测距高度
 *
 * ① 平滑 / 跟随的权衡(R 与 Q 之比):
 *      SENSOR_TOF_MEAS_STD 调大 => 更信模型: 高度/速度更平滑, 但跟随更慢
 *      SENSOR_TOF_ACC_STD  调大 => 更信量测: 跟随更快, 但高度/速度更抖
 * ② 反射面突变(地面 <-> 桌子上空/台阶): 跳变先吸收到基准偏置(输出不跳变),
 *    再按 SENSOR_TOF_TAU_OFFSET 缓慢跟随新反射面
 * ③ 保护: 倾角过大时限制 cos 放大; 量程外的测距值直接丢弃
 * ======================================================================= */
#define SENSOR_TOF_MEAS_STD        5.0f    //高度量测噪声标准差(mm): 测距噪声+倾角补偿残差   [大=平滑]
#define SENSOR_TOF_ACC_STD         400.0f   //垂直加速度过程噪声标准差(mm/s^2)             [大=跟随快]
#define SENSOR_TOF_MAX_INNOV       300.0f   //突变判据(mm): 高度偏离超此值判为换了反射面   [小=更敏感]
#define SENSOR_TOF_TAU_OFFSET      2.0f     //突变后跟随新反射面的时间常数(s)              [小=适应快]
#define SENSOR_TOF_RANGE_MAX       4000.0f  //有效量程(mm): 0(未出数据)/超量程按无效量测丢弃
#define SENSOR_TOF_MIN_COS_TILT    0.174f   //cos(80°): 倾角过大/倒扣时的放大上限, 防高度炸开

void TOF_Update(float *TOF_Height_Out, float *TOF_Height_Speed_Out, float TOF_Distence,
                      float Cos_Tilt, float Dt)
{
    static float Height_Last;          //上一帧倾角补偿后的高度(mm), 供粗检用
    static float Speed_Last;           //上一帧原始差分速度(mm/s), 供粗检用
    static float Height_NoFilter;      //状态量1: 高度估计(mm), 即原来的积分高度
    static float Speed_NoFilter;       //状态量2: 速度估计(mm/s)
    static float Height_Offset;        //反射面基准偏置(mm): 突变先吸收到这里(输出不跳变), 再缓慢泄放
    static float P00, P01, P10, P11;   //状态协方差矩阵 P(k|k)
    static uint8_t KF_Inited;          //首个有效量测用来初始化状态, 避免从0慢慢收敛
    
    
    Limit_float(&Dt, 1, 0.001f);
    
    float cos_tilt = Cos_Tilt;
    if (cos_tilt < SENSOR_TOF_MIN_COS_TILT) cos_tilt = SENSOR_TOF_MIN_COS_TILT;
    float Height = TOF_Distence * cos_tilt;
    
    float Speed = (Height - Height_Last) / Dt;   //原始差分速度(mm/s), 噪声大, 只用来粗检
    
    /* 量测有效性: 参考值10/超量程(如65535)直接丢弃, 不做任何修正, 防止把高度带飞 */
    uint8_t Meas_Valid = (TOF_Distence > 10 && TOF_Distence <= SENSOR_TOF_RANGE_MAX) ? 1 : 0;
    
    /* 反射面突变粗检(原判据形式): 原始差分速度相对上一帧发生大跳变 */
    uint8_t Surface_Changed = (fabsf(Speed - Speed_Last) > SENSOR_TOF_MAX_INNOV / Dt) ? 1 : 0;
    
    /* ========================= 卡尔曼滤波 =========================
     * x = [高度; 速度], F = [1 Dt; 0 1], H = [1 0](量测即倾角补偿后的高度) */
    
    //0. 首次拿到有效量测: 用它初始化状态与协方差(原来是积分高度从0慢慢收敛)
    if(!KF_Inited && Meas_Valid)
    {
        Height_NoFilter = Height;
        Speed_NoFilter  = 0;
        Height_Offset   = 0;
        P00 = SENSOR_TOF_MEAS_STD * SENSOR_TOF_MEAS_STD;            //高度方差: 量测噪声量级
        P11 = SENSOR_TOF_ACC_STD * SENSOR_TOF_ACC_STD * Dt * Dt;    //速度方差: 单帧速度噪声量级
        P01 = 0; P10 = 0;
        KF_Inited = 1;
    }
    
    //1. 状态预测 x(k|k-1) = F·x(k-1|k-1): 高度按速度外推, 速度不变
    Height_NoFilter += Speed_NoFilter * Dt;
    
    //2. 协方差预测 P(k|k-1) = F·P(k-1|k-1)·F' + Q, Q由加速度白噪声离散化
    float Q_Pos   = 0.25f * Dt*Dt*Dt*Dt * SENSOR_TOF_ACC_STD * SENSOR_TOF_ACC_STD;
    float Q_Cross = 0.5f  * Dt*Dt*Dt    * SENSOR_TOF_ACC_STD * SENSOR_TOF_ACC_STD;
    float Q_Speed =         Dt*Dt       * SENSOR_TOF_ACC_STD * SENSOR_TOF_ACC_STD;
    float P_Pos   = P00 + Dt * (P10 + P01) + Dt*Dt * P11 + Q_Pos;
    float P_Cross = P01 + Dt * P11 + Q_Cross;
    float P_Speed = P11 + Q_Speed;
    
    //3. 量测更新: S = H·P·H' + R, K = P·H'/S; 量测 = 倾角补偿高度 + 反射面基准偏置
    if(Meas_Valid)
    {
        float S = P_Pos + SENSOR_TOF_MEAS_STD * SENSOR_TOF_MEAS_STD;    //新息协方差
        float Innovation = (Height + Height_Offset) - Height_NoFilter;  //新息 z - H·x(k|k-1)
        float K_Pos   = P_Pos   / S;                                    //高度增益
        float K_Speed = P_Cross / S;                                    //速度增益
        
        /* 剔除反射面突变(地面<->桌子上空/台阶): 把跳变吸收到基准偏置里,
         * 状态与输出都不跳变; 下一帧新息≈0不会一直剔除(否则速度会卡死),
         * 被吸收的偏置再按 SENSOR_TOF_TAU_OFFSET 缓慢泄放 => 缓慢跟随新的反射面 */
        if(Surface_Changed || (fabsf(Innovation) > SENSOR_TOF_MAX_INNOV))
        {
            Height_Offset -= Innovation;
            Meas_Valid = 0;
        }
        else
        {
            Height_NoFilter += K_Pos   * Innovation;
            Speed_NoFilter  += K_Speed * Innovation;
            
            //P(k|k) = (I - K·H)·P(k|k-1)
            P00 = (1.0f - K_Pos) * P_Pos;
            P01 = (1.0f - K_Pos) * P_Cross;
            P10 = P01;
            P11 = P_Speed - K_Speed * P_Cross;
        }
    }
    
    if(!Meas_Valid) //量测被剔除: 只保留预测结果, 等下一个量测
    {
        P00 = P_Pos; P01 = P_Cross; P10 = P_Cross; P11 = P_Speed;
    }
       
    
    /* 反射面基准偏置缓慢泄放时间常数 SENSOR_TOF_TAU_OFFSET:
     * 泄放量直接补到状态高度上, 不经过量测更新, 所以不会产生"假速度" */
    float Offset_Last = Height_Offset;
    Filter_LowPass(0.0f, &Height_Offset, SENSOR_TOF_TAU_OFFSET, Dt);
    Height_NoFilter += (Height_Offset - Offset_Last);
    
    /* 输出 */
    *TOF_Height_Out       = Height_NoFilter;
    *TOF_Height_Speed_Out = Speed_NoFilter;
    
    Height_Last = Height;
    Speed_Last  = Speed;
}



#define SENSOR_FLOW_RAD_PER_COUNT   0.001936f ;     /* rad/count: 实测标定(0.0209/9.36), 原地转动 flow≈body */
#define SENSOR_FLOW_SQUAL_MIN      90               /* 表面质量低于此判无效(0~169, >50可靠) */
#define SENSOR_FLOW_TAU            0.2f             /* 输出一阶低通时间常数(s) */
#define SENSOR_FLOW_Max_Vel        4000             /* 最大速度mm/s */    

void Sensor_Flow_Update(float *Vx_out, float *Vy_out, 
                        float Pix_Vx, float Pix_Vy, float Squal,
                        float Height_mm, float Body_Gyro_X_rad, float Body_Gyro_Y_rad, 
                        float dt)
{
    if (dt < 0.001f) dt = 0.001f;   // 兜底

    /* 光流像素速率 -> 角速率(rad/s) */
    float Flow_Vx_rad = Sensor.Flow_Pix_Vx * SENSOR_FLOW_RAD_PER_COUNT;
    float Flow_Vy_rad = Sensor.Flow_Pix_Vy * SENSOR_FLOW_RAD_PER_COUNT;

    /* 表面质量低(离地太近/纹理差/失锁)时不更新, 速度向0缓慢衰减 */
    if ((Squal < SENSOR_FLOW_SQUAL_MIN) || (Height_mm < 50.0f))
    {
        Filter_LowPass(0, Vx_out, SENSOR_FLOW_TAU, dt);
        Filter_LowPass(0, Vy_out, SENSOR_FLOW_TAU, dt);
    }
    else
    {
        Filter_LowPass(((Flow_Vx_rad - Body_Gyro_X_rad) * Height_mm), Vx_out, SENSOR_FLOW_TAU, dt);
        Filter_LowPass(((Flow_Vy_rad - Body_Gyro_Y_rad) * Height_mm), Vy_out, SENSOR_FLOW_TAU, dt);
    }
    Limit_float(Vx_out, SENSOR_FLOW_Max_Vel, -SENSOR_FLOW_Max_Vel);
    Limit_float(Vy_out, SENSOR_FLOW_Max_Vel, -SENSOR_FLOW_Max_Vel);
}
