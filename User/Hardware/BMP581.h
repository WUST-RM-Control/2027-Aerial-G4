#ifndef __BMP581_H__
#define __BMP581_H__

#include "main.h"
#include "Parameter.h"   /* 海平面气压 / ID 重试 / INT 兜底 / 气压量程 见 Parameter.h 第 4 节 */

/*==============================================================================
 *  BMP581 气压计 (V3 控制板 U8)
 *    总线: SPI1  SCK=PA5  MISO=PA6  MOSI=PA7
 *    CS  : PB0 (BPS_CS)
 *    INT : PB1 (BPS_INT, EXTI1 下降沿)  <- 数据就绪中断
 *
 *  与旧 SPA06(I2C) 的区别: 数据已经片内补偿, 直接输出 24bit 温度/气压,
 *  不需要像 SPL07 那样读标定系数再做二阶补偿。
 *============================================================================*/
#define BMP581_SPI              hspi1
#define BMP581_CS(x)            HAL_GPIO_WritePin(BPS_CS_GPIO_Port, BPS_CS_Pin, \
                                                  (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#define BMP581_CHIP_ID_VALUE    0x50

typedef enum
{
    BMP581_Reg_CHIP_ID     = 0x01,
    BMP581_Reg_REV_ID      = 0x02,
    BMP581_Reg_INT_CONFIG  = 0x14,   /* bit3 en, bit2 od, bit1 pol, bit0 mode */
    BMP581_Reg_INT_SOURCE  = 0x15,   /* bit0 drdy_en                          */
    BMP581_Reg_TEMP_XLSB   = 0x1D,   /* 0x1D~0x1F: 温度 24bit                 */
    BMP581_Reg_PRESS_XLSB  = 0x20,   /* 0x20~0x22: 气压 24bit                 */
    BMP581_Reg_INT_STATUS  = 0x27,   /* bit0 drdy                             */
    BMP581_Reg_STATUS      = 0x28,
    BMP581_Reg_OSR_CONFIG  = 0x36,   /* [2:0]osr_t [5:3]osr_p [6]press_en     */
    BMP581_Reg_ODR_CONFIG  = 0x37,   /* [6:2]odr  [1:0]powermode             */
    BMP581_Reg_CMD         = 0x7E,   /* 0xB6 = 软复位                         */
} BMP581_Reg_EnumTypedef;

typedef struct
{
    float Pressure;             /* Pa                      */
    float Temperature;          /* 摄氏度                  */
    float Altitude;             /* 未滤波绝对海拔(m)       */
    float Relative_Height;      /* 相对上电零点的高度(m)   */
    int32_t Relative_Height_mm; /* 相对上电零点的高度(mm)  */

    volatile uint32_t Int_Count;        /* INT 引脚中断次数(判连线/配置)   */
    volatile uint32_t Poll_Count;       /* 无 INT 时的兜底轮询次数         */
    uint32_t Sample_Count;              /* 有效样本数                      */
    uint8_t  Chip_ID;                   /* 应为 0x50                       */
    uint8_t  Ready;                     /* 1 = 已识别并配置成功            */
} BMP581_StructTypedef;

extern BMP581_StructTypedef BMP581;

/* 初始化: 软复位 -> 配置 OSR/ODR/INT -> 进入 normal 连续测量 */
void BMP581_Init(void);
/* 读一次数据: 只在 INT 标志置位(或超时兜底)时真正走 SPI, 返回 1=本次有新样本 */
uint8_t BMP581_Read_Data(void);
/* 以当前海拔为新零点(相对高度归零) */
void BMP581_Reset_Height(void);
/* EXTI1 中断(PB1)里调用: 置数据就绪标志 */
void BMP581_INT_Callback(void);

/* 寄存器读写(供调试/扩展) */
uint8_t BMP581_Read_Reg(BMP581_Reg_EnumTypedef Reg);
void    BMP581_Read_RegList(BMP581_Reg_EnumTypedef Reg, uint8_t Num, uint8_t *Read_Data);
void    BMP581_Write_Reg(BMP581_Reg_EnumTypedef Reg, uint8_t Write_Data);

#endif
