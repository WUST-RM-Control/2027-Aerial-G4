#ifndef __QMI8658A__
#define __QMI8658A__

#include "main.h"

#define GRAVITY 9.8f   

typedef enum
{
    QMI8658A_Reg_WHO_AM_I       = 0x00,
    QMI8658A_Reg_REVISION_ID    = 0x01,
    QMI8658A_Reg_CTRL1          = 0x02,
    QMI8658A_Reg_CTRL2          = 0x03,
    QMI8658A_Reg_CTRL3          = 0x04,
    QMI8658A_Reg_CTRL5          = 0x06,
    QMI8658A_Reg_CTRL6          = 0x07,
    QMI8658A_Reg_CTRL7          = 0x08,
    QMI8658A_Reg_CTRL8          = 0x09,
    QMI8658A_Reg_CTRL9          = 0x0A,
    QMI8658A_Reg_FIFO_WTM_TH    = 0x13,
    QMI8658A_Reg_FIFO_CTRL      = 0x14,
    QMI8658A_Reg_FIFO_SMPL_CNT  = 0x15,
    QMI8658A_Reg_FIFO_STATUS    = 0x16,
    QMI8658A_Reg_FIFO_DATA      = 0x17,
    QMI8658A_Reg_STATUSINT      = 0x2D,
    QMI8658A_Reg_STATUS0        = 0x2E,
    QMI8658A_Reg_STATUS1        = 0x2F,
    QMI8658A_Reg_TEMP_L         = 0x33,
    QMI8658A_Reg_TEMP_H         = 0x34,
    QMI8658A_Reg_AX_L           = 0x35,
    QMI8658A_Reg_AX_H           = 0x36,
    QMI8658A_Reg_AY_L           = 0x37,
    QMI8658A_Reg_AY_H           = 0x38,
    QMI8658A_Reg_AZ_L           = 0x39,
    QMI8658A_Reg_AZ_H           = 0x3A,
    QMI8658A_Reg_GX_L           = 0x3B,
    QMI8658A_Reg_GX_H           = 0x3C,
    QMI8658A_Reg_GY_L           = 0x3D,
    QMI8658A_Reg_GY_H           = 0x3E,
    QMI8658A_Reg_GZ_L           = 0x3F,
    QMI8658A_Reg_GZ_H           = 0x40,
    QMI8658A_Reg_RESET          = 0x60,
    
} QMI8658A_Reg_EnumTypedef;


typedef __PACKED_STRUCT
{
    float Accel[3];
    float Gyro[3];

    float Temperature;
} QMI8658A_StructTypedef;

//QMI8658A数据结构体
extern QMI8658A_StructTypedef QMI8658A;


//QMI8658A初始化
void QMI8658A_Init(void);
//QMI8658A读全部数据
void QMI8658A_Read_Data(void);


//QMI8658A读单个寄存器
uint8_t QMI8658A_Read_Reg(QMI8658A_Reg_EnumTypedef Reg);
//QMI8658A读多个寄存器
void QMI8658A_Read_RegList(QMI8658A_Reg_EnumTypedef Reg, uint8_t Num, uint8_t *Read_Data);
//QMI8658A写单个寄存器
void QMI8658A_Write_Reg(QMI8658A_Reg_EnumTypedef Reg, uint8_t Write_Data);

#endif
