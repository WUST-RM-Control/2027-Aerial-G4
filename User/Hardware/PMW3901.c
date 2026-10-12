#include "PMW3901.h"
#include "DWT.h"

//PMW3901数据结构体
PMW3901_StructTypedef PMW3901;

//PMW3901通信配置(挂在SPI1, CS=PB0)
#define PMW3901_SPI            hspi1
#define PMW3901_CS_Enable()    HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_RESET)
#define PMW3901_CS_Disable()   HAL_GPIO_WritePin(PMW_CS_GPIO_Port, PMW_CS_Pin, GPIO_PIN_SET)

//µs级延时(挂起/耗时操作, 请勿在中断中调用)
#define PMW3901_Delay_us(us)   DWT_Delay((us) / 1000000.0f)

//PMW3901读单个寄存器
//注意:PMW3901的寄存器首字节bit7=0表示"读"(与多数传感器相反)
uint8_t PMW3901_Read_Reg(PMW3901_Reg_EnumTypedef Reg)
{
    uint8_t addr = ((uint8_t)Reg) & 0x7F;   // 读命令
    uint8_t dummy = 0;
    uint8_t data = 0;

    PMW3901_CS_Enable();
    PMW3901_Delay_us(50);
    HAL_SPI_Transmit(&PMW3901_SPI, &addr, 1, 1000);
    PMW3901_Delay_us(100);                    // 等待MISO有效(tSRAD>=35us)
    HAL_SPI_TransmitReceive(&PMW3901_SPI, &dummy, &data, 1, 1000);
    PMW3901_Delay_us(50);
    PMW3901_CS_Disable();

    return data;
}

//PMW3901写单个寄存器
//注意:PMW3901的寄存器首字节bit7=1表示"写"(与多数传感器相反)
void PMW3901_Write_Reg(PMW3901_Reg_EnumTypedef Reg, uint8_t Write_Data)
{
    uint8_t addr = ((uint8_t)Reg) | 0x80;     // 写命令

    PMW3901_CS_Enable();
    PMW3901_Delay_us(50);
    HAL_SPI_Transmit(&PMW3901_SPI, &addr, 1, 1000);
    PMW3901_Delay_us(50);
    HAL_SPI_Transmit(&PMW3901_SPI, &Write_Data, 1, 1000);
    PMW3901_Delay_us(50);
    PMW3901_CS_Disable();
}

//初始化寄存器表(前半部分, 来自crazyflie实机验证序列, 注意0x7F为分页寄存器, 顺序不可乱)
static const uint8_t PMW3901_Init_RegListA[][2] =
{
    {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00},
    {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08},
    {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F},
    {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18},
    {0x7F, 0x09}, {0x4F, 0xAF}, {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77},
                 {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50},
    {0x7F, 0x0A}, {0x45, 0x60},
    {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78}, {0x4B, 0x78},
                 {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF}, {0x65, 0x1F},
    {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08}, {0x63, 0x70},
    {0x7F, 0x15}, {0x48, 0x48},
    {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80},
    {0x7F, 0x10}, {0x5B, 0x02},
    {0x7F, 0x07}, {0x40, 0x41},
    {0x70, 0x00},   // 到这里延时10ms再继续后半段
};

//初始化寄存器表(后半部分)
static const uint8_t PMW3901_Init_RegListB[][2] =
{
    {0x32, 0x44},
    {0x7F, 0x07}, {0x40, 0x40},
    {0x7F, 0x06}, {0x62, 0xF0}, {0x63, 0x00},
    {0x7F, 0x0D}, {0x48, 0xC0}, {0x6F, 0xD5},
    {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80},
    {0x7F, 0x00}, {0x5A, 0x10}, {0x54, 0x00},
};

//PMW3901初始化
void PMW3901_Init(void)
{
    // 0. 把SPI1切到模式3(CPOL=1,CPHA=1), 并把波特率降到2MHz以内
    //    PMW3901手册要求: 4-Wire SPI @ 2MHz, SPI Mode 3
    HAL_SPI_DeInit(&PMW3901_SPI);
    PMW3901_SPI.Init.Mode = SPI_MODE_MASTER;
    PMW3901_SPI.Init.Direction = SPI_DIRECTION_2LINES;
    PMW3901_SPI.Init.DataSize = SPI_DATASIZE_8BIT;
    PMW3901_SPI.Init.CLKPolarity = SPI_POLARITY_HIGH;            // CPOL=1
    PMW3901_SPI.Init.CLKPhase = SPI_PHASE_2EDGE;                 // CPHA=1  => SPI Mode 3
    PMW3901_SPI.Init.NSS = SPI_NSS_SOFT;
    PMW3901_SPI.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128; // 170MHz/128 ≈ 1.33MHz
    PMW3901_SPI.Init.FirstBit = SPI_FIRSTBIT_MSB;
    if (HAL_SPI_Init(&PMW3901_SPI) != HAL_OK)
    {
        Error_Handler();
    }

    // 1. CS拉高, 等待上电稳定
    PMW3901_CS_Disable();
    HAL_Delay(40);

    // 2. 产生一次CS复位脉冲
    PMW3901_CS_Enable();
    HAL_Delay(2);
    PMW3901_CS_Disable();
    HAL_Delay(2);

    // 3. 读取产品ID(0x00=0x49, 0x5F=0x49取反=0xB6)
    uint8_t chipId = PMW3901_Read_Reg(PMW3901_Reg_Product_ID);
    uint8_t invId  = PMW3901_Read_Reg((PMW3901_Reg_EnumTypedef)0x5F);

    if ((chipId == 0x49) && (invId == 0xB6))
    {
        // 4. 上电复位
        PMW3901_Write_Reg((PMW3901_Reg_EnumTypedef)0x3A, 0x5A);
        HAL_Delay(5);

        // 5. 空读一次运动寄存器, 清除上电产生的运动标志
        PMW3901_Read_Reg(PMW3901_Reg_Motion);
        PMW3901_Read_Reg(PMW3901_Reg_Delta_X_L);
        PMW3901_Read_Reg(PMW3901_Reg_Delta_X_H);
        PMW3901_Read_Reg(PMW3901_Reg_Delta_Y_L);
        PMW3901_Read_Reg(PMW3901_Reg_Delta_Y_H);
        HAL_Delay(1);

        // 6. 写入初始化寄存器表
        for (uint8_t i = 0; i < (sizeof(PMW3901_Init_RegListA) / sizeof(PMW3901_Init_RegListA[0])); i++)
        {
            PMW3901_Write_Reg((PMW3901_Reg_EnumTypedef)PMW3901_Init_RegListA[i][0], PMW3901_Init_RegListA[i][1]);
        }
        HAL_Delay(10);
        for (uint8_t i = 0; i < (sizeof(PMW3901_Init_RegListB) / sizeof(PMW3901_Init_RegListB[0])); i++)
        {
            PMW3901_Write_Reg((PMW3901_Reg_EnumTypedef)PMW3901_Init_RegListB[i][0], PMW3901_Init_RegListB[i][1]);
        }
    }
}

//PMW3901突发读全部运动数据(寄存器0x16一次性流式读出12字节)
void PMW3901_Read_Data(void)
{
    uint8_t addr = PMW3901_Reg_Motion_Burst;   // bit7=0 => 读
    uint8_t dummy[12] = {0};
    uint8_t buf[12] = {0};

    PMW3901_CS_Enable();
    PMW3901_Delay_us(50);
    HAL_SPI_Transmit(&PMW3901_SPI, &addr, 1, 1000);
    PMW3901_Delay_us(100);
    HAL_SPI_TransmitReceive(&PMW3901_SPI, dummy, buf, 12, 1000);
    PMW3901_Delay_us(50);
    PMW3901_CS_Disable();

    memcpy((uint8_t *)&PMW3901, buf, 12);

    // 曝光时间是高字节在前, 与其它数据字节序相反, 需要交换
    PMW3901.Shutter = (uint16_t)((PMW3901.Shutter >> 8) & 0x0FF) |
                      (uint16_t)((PMW3901.Shutter & 0x0FF) << 8);
}
