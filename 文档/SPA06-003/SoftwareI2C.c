/*******************************************************************************
* Copyright (C), 2000-2016,  Electronic Technology Co., Ltd.
* FileName: SoftwareI2C.c
* Author: Li.wen
* Version:
* Date: 2016-1-8     
* Description: MCU IO controlled I2C program
* History:       
*    1. Date: 2016-1-8
*       Author: Li.wen
*       Modification: create
*    2.
* Others:           //
*******************************************************************************/

#ifndef SOFTWARE_I2C_C
#define SOFTWARE_I2C_C

#include "SoftwareI2C.h"

#define PIN_SCL P0_4
#define PIN_SDA P0_3
#define SCL_OUTPUT() P0DIR|=(1<<4)
#define SDA_INPUT() P0DIR&=~(1<<3)
#define SDA_OUTPUT() P0DIR|=(1<<3)

#define DelayTick 0

void SoftI2C_delay(uint8 i)
{
    while(i--);
}

void SoftI2C_stop()
{
    PIN_SCL = 1;
    SoftI2C_delay(DelayTick);
    PIN_SDA = 1;
    SoftI2C_delay(DelayTick);
}
void SoftI2C_start()
{
    SCL_OUTPUT();
    SDA_OUTPUT();
    PIN_SDA = 1;
    SoftI2C_delay(DelayTick);
    PIN_SCL = 1;
    PIN_SDA = 0;
    SoftI2C_delay(DelayTick);
    PIN_SCL = 0;
}
void SoftI2C_write_byte(uint8 value)
{
    uint8 i;
    uint8 result = 0;
    i = 8;
    while (i)
    {
        if (value & 0x80)
            PIN_SDA = 1;
        else
            PIN_SDA = 0;
        SoftI2C_delay(DelayTick);
        PIN_SCL = 1;
        i--;
        value <<= 1;
        SoftI2C_delay(DelayTick);
        PIN_SCL = 0;
        SoftI2C_delay(DelayTick);
    }
    SDA_INPUT();        //input
    SoftI2C_delay(DelayTick);

    PIN_SCL = 1;
    SoftI2C_delay(DelayTick);
    if (PIN_SDA)
    {
        result = 1;
    }
    PIN_SCL = 0;
    PIN_SDA = 0;
    SDA_OUTPUT();   //output
}
uint8 SoftI2C_read_byte(uint8 ack)
{
    uint8 i;
    uint8 result = 0;
    i = 8;
    SDA_INPUT();        //input
    while (i)
    {
        result <<= 1;
        PIN_SCL = 1;
        i--;
        SoftI2C_delay(DelayTick);
        if (PIN_SDA)
        {
            result |= 0x01;
        }
        PIN_SCL = 0;

        SoftI2C_delay(DelayTick);
    }
    SDA_OUTPUT();   //output
    if (ack)
        PIN_SDA = 1;
    else
        PIN_SDA = 0;
    SoftI2C_delay(DelayTick);
    PIN_SCL = 1;
    SoftI2C_delay(DelayTick);
    PIN_SCL = 0;
    PIN_SDA = 0;
    SoftI2C_delay(3);
    return result;
}

#endif

