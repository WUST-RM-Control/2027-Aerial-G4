#include "QMI8658A.h"
#include "DWT.h"

//QMI8658A数据结构体
QMI8658A_StructTypedef QMI8658A;


//QMI8658A通信配置
//  SPI2 内核时钟 170MHz(PCLK1) / 16 = 10.625MHz: 读一次 15 字节 ≈ 11us
//  (原 PSC=32 是 5.3MHz ≈ 23us, 4kHz 下白白多占 5% CPU)
#define QMI8658A_SPI        hspi2
#define QMI8658A_CS(PIN_Status)    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, PIN_Status)

//QMI8658A读单个寄存器
uint8_t QMI8658A_Read_Reg(QMI8658A_Reg_EnumTypedef Reg)
{
    uint8_t tx[2] = {0x80 | Reg, 0};
    uint8_t rx[2] = {0};
    
    QMI8658A_CS(0);
    HAL_SPI_TransmitReceive(&QMI8658A_SPI, tx, rx, 2, 1000);
    QMI8658A_CS(1);
    
    return rx[1];
}

//QMI8658A读多个寄存器
void QMI8658A_Read_RegList(QMI8658A_Reg_EnumTypedef Reg, uint8_t Num, uint8_t *Read_Data)
{
    uint8_t tx[16] = {0x80 | Reg, 0};
    uint8_t rx[16] = {0};

    QMI8658A_CS(0);
    HAL_SPI_TransmitReceive(&QMI8658A_SPI, tx, rx, Num+1, 1000);
    QMI8658A_CS(1);
    
    memcpy(Read_Data, &rx[1], Num);
}

//QMI8658A写单个寄存器
void QMI8658A_Write_Reg(QMI8658A_Reg_EnumTypedef Reg, uint8_t Write_Data)
{
    uint8_t tx[2] = {0x7F&Reg, Write_Data};
    uint8_t rx[2] = {0};

    QMI8658A_CS(0);
    HAL_SPI_TransmitReceive(&QMI8658A_SPI, tx, rx, 2, 1000);
    QMI8658A_CS(1);
}

//配置寄存器表
uint8_t QMI8658A_Init_RegList[][2] = 
{   
    {QMI8658A_Reg_CTRL1     , 0x60}, // 开启SPI地址自动递增,MSB
    {QMI8658A_Reg_CTRL7     , 0x00}, // 关闭加速度计+陀螺仪+温度传感器
    /* ODR 档位: 0=7174.4Hz 1=3587.2Hz 2=1793.6Hz 3=896.8Hz ...
     * 主循环 2kHz(500us) 下取档位 1(3587.2Hz): 每拍 1.79 个新样本,
     *   与原来 4kHz/7174 的比例一致; 且芯片 LPF 截止频率 = 百分比 × ODR
     *   自动从 959Hz 降到 ≈480Hz, 对 2kHz 采样的抗混叠更充分(1~1.8kHz 折叠带压 12~23dB)。
     * 取 0(7174.4Hz) 的话就是 3.59 个样本/拍, 采样比偏大且抗混叠变弱;
     * 取 2(1793.6Hz) 则一拍不足 1 个样本, 同一样本会被重复读, D 项会被喂成阶梯。
     * 代价: 样本"年龄"抖动 0~279us(档位 0 是 0~139us)。 */
    {QMI8658A_Reg_CTRL2     , 0x21}, // 加速度计：量程±8g(aFS=2)，输出数据率3587.2Hz(aODR=1)
    {QMI8658A_Reg_CTRL3     , 0x71}, // 陀螺仪：量程±2048dps(gFS=7, 最大量程)，输出数据率3587.2Hz(gODR=1)
    // CTRL5 内置低通：bit0=加速度计LPF使能, bit[2:1]=加速度计档位, bit4=陀螺LPF使能, bit[6:5]=陀螺档位
    //   档位 = ODR的百分比(0=2.66% 1=3.63% 2=5.39% 3=13.37%)
    //   0x77 = 两路都开、档位3(13.37%) → 3587.2Hz*13.37% ≈ 480Hz，延迟约0.33ms
    //          (截止频率 = 百分比 × ODR, 换 ODR 档位时它会跟着变)
    //          带宽留给软件陷波/低通去管, 芯片侧只负责挡掉 Nyquist(1kHz) 以上的东西
    //          ★ 0x00(全关) 只在需要看“原始振动波形/频谱”时临时用:
    //            带宽无限制 → 加速度计/陀螺的高频振动会直接进 EKF 观测,
    //            残差超卡方阈值后整帧跳过加速度计修正 → 纯陀螺积分漂移
    //   其他：0x03=只开加速度计(陀螺无LPF)  0x30=只开陀螺  0x33=两路档位1  0x00=全关
    {QMI8658A_Reg_CTRL5     , 0x77},
    {QMI8658A_Reg_CTRL8     , 0x01}, // 开启温度传感器
    {QMI8658A_Reg_CTRL7     , 0x03}, // 开启加速度计+陀螺仪+温度传感器};
};

//QMI8658A初始化
void QMI8658A_Init(void)
{
    //写入初始化寄存器表
    uint8_t reg_num = sizeof(QMI8658A_Init_RegList) / sizeof(QMI8658A_Init_RegList[0]);
    for(uint8_t i = 0; i < reg_num; i++)
    {
        QMI8658A_Write_Reg(QMI8658A_Init_RegList[i][0], QMI8658A_Init_RegList[i][1]);
    }
}
 
//QMI8658A读全部数据
void QMI8658A_Read_Data(void)
{
    __PACKED_STRUCT
    {
        uint16_t Tempture;
        int16_t AX;
        int16_t AY;
        int16_t AZ;
        int16_t GX;
        int16_t GY;
        int16_t GZ;;
    } RAW_Data;
    
    QMI8658A_Read_RegList(QMI8658A_Reg_TEMP_L, 14, (uint8_t*)&RAW_Data);    
    
    /* 根据量程解包 (CTRL2=±8g, CTRL3=±2048dps)
     * ±8g: 4096 LSB/g, 1 LSB ≈ 0.0024 m/s² ≈ 0.24mg(≈0.014° 倾角)。
     * EKF 只看重力方向, 这点量化噪声可忽略; 若需要更大抗冲击范围,
     * 把 CTRL2 改回 0x30(±16g) 并同步改下面系数为 16.0f。 */
    QMI8658A.Temperature =  RAW_Data.Tempture / 256.0f;
    QMI8658A.Accel[0] = 8.0f * GRAVITY * RAW_Data.AX / 32768.0f;
    QMI8658A.Accel[1] = 8.0f * GRAVITY * RAW_Data.AY / 32768.0f;
    QMI8658A.Accel[2] = 8.0f * GRAVITY * RAW_Data.AZ / 32768.0f;
    QMI8658A.Gyro[0]  = 2048.0f * PI / 180.0f * RAW_Data.GX / 32768.0f;
    QMI8658A.Gyro[1]  = 2048.0f * PI / 180.0f * RAW_Data.GY / 32768.0f;
    QMI8658A.Gyro[2]  = 2048.0f * PI / 180.0f * RAW_Data.GZ / 32768.0f;
}
