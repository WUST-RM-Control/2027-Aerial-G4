#ifndef __BMI088_H__
#define __BMI088_H__

#include "main.h"
#include "Parameter.h"   /* 量程 / ID 重试 / INT 兜底周期 见 Parameter.h 第 4 节 */

/*==============================================================================
 *  BMI088 六轴 IMU (V3 控制板 U34, 双片 SPI)
 *    总线: SPI2  SCK=PB13  MISO=PB14  MOSI=PB15
 *    加速度计: CS=PB12 (IMU_CS_ACC)   INT1=PA9  (IMU_INT_ACC,  EXTI9  下降沿)
 *    陀螺仪  : CS=PA8  (IMU_CS_GYRO)  INT3=PA10 (IMU_INT_GYRO, EXTI10 下降沿)
 *
 *  与旧 QMI8658A(单 CS + 单 INT) 的区别:
 *    - 两个 die 各有自己的 CS, 同一组 SPI 引脚;
 *    - INT1(加速度) 与 INT3(陀螺) 都是数据就绪中断, 各自独立;
 *    - 加速度计 SPI 读时序比陀螺多一个 dummy 字节(见 BMI088.c)。
 *============================================================================*/
#define BMI088_SPI              hspi2
#define BMI088_CS_ACC(x)        HAL_GPIO_WritePin(IMU_CS_ACC_GPIO_Port, IMU_CS_ACC_Pin, \
                                                  (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define BMI088_CS_GYRO(x)       HAL_GPIO_WritePin(IMU_CS_GYRO_GPIO_Port, IMU_CS_GYRO_Pin, \
                                                  (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* 量程换算系数: 量程本身(BMI088_ACCEL_RANGE_G / BMI088_GYRO_RANGE_DPS)
 * 在 Parameter.h 里, 改量程时**必须**同时改 BMI088.c 的 ACC_RANGE / GYRO_RANGE 寄存器 */
#define BMI088_ACCEL_LSB_PER_G   (32768.0f / BMI088_ACCEL_RANGE_G)
#define BMI088_GYRO_LSB_PER_DPS  (32768.0f / BMI088_GYRO_RANGE_DPS)

typedef struct
{
    float Accel[3];         /* m/s^2, 芯片坐标系(未做安装方向/零偏校准) */
    float Gyro[3];          /* rad/s, 芯片坐标系                        */
    float Temperature;      /* 摄氏度                                   */
    volatile uint32_t Temp_Tick;       /* 温度最近一次更新的时刻(ms), 加热控温判新鲜度 */

    volatile uint32_t Acc_Int_Count;   /* INT1(加速度) 中断次数            */
    volatile uint32_t Gyro_Int_Count;  /* INT3(陀螺)   中断次数            */
    volatile uint32_t Acc_Poll_Count;  /* 无 INT 时兜底读取次数            */
    volatile uint32_t Gyro_Poll_Count;

    /* ---------------- 样本时间戳(提高姿态解算精度用) ----------------
     * Ts_Cnt   : 【EXTI 里写】样本就绪那一刻的 DWT 周期数(32bit, 回绕安全)
     * Sample_Cyc: 【任务里写】当前这份数据对应的周期数
     * Dt_Cyc   : 相邻两个样本的周期差 -> EKF 用 DWT_Cyc_To_S() 换成真实 dt
     * Dt_Us    : 同一间隔的微秒值, 只用于调试查看(分辨率 1us)
     * Seq      : 已读样本计数, 外层靠“序号变了”判断有没有新样本 */
    volatile uint32_t Acc_Ts_Cnt;
    volatile uint32_t Gyro_Ts_Cnt;
    uint32_t Acc_Sample_Cyc;
    uint32_t Gyro_Sample_Cyc;
    uint32_t Acc_Dt_Cyc;
    uint32_t Gyro_Dt_Cyc;
    uint32_t Acc_Dt_Us;
    uint32_t Gyro_Dt_Us;
    uint32_t Acc_Seq;
    uint32_t Gyro_Seq;

    uint8_t Chip_ID_Acc;    /* 0x1E */
    uint8_t Chip_ID_Gyro;   /* 0x0F */
    uint8_t Ready;          /* 1 = 两颗 die 都识别并配置成功 */
} BMI088_StructTypedef;

extern BMI088_StructTypedef BMI088;

/* 初始化: 复位 -> 配置量程/带宽/ODR -> 配置 INT1/INT3 数据就绪输出 */
void BMI088_Init(void);
/* 读一次: 只有对应 INT 标志置位(或超时兜底)的那个 die 才走 SPI */
void BMI088_Read_Data(void);
/* EXTI 回调: 只置标志/计数, 不做 SPI */
void BMI088_INT_Acc_Callback(void);
void BMI088_INT_Gyro_Callback(void);

#endif
