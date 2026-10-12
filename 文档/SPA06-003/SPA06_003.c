/*******************************************************************************
* Copyright (C), 2000-2016,  Electronic Technology Co., Ltd.
* FileName: SPL07_01.c
* Author: Li.wen
* Version:
* Date: 2015-11-30     
* Description:Define the operation interface of SPL07-01
* History:
*    1. Date: 2015-11-30
*       Author: Li.wen
*       Modification: create
*    2. Date£º2016-10-12
		Author£ºlan.xing
		Modification£ºmodified the calibrate coefficient
* Others:           
*******************************************************************************/
#ifndef SPL07_003_C
#define SPL07_003_C

#include "spl07_003.h"
#include "SoftwareI2C.h"

static struct spl07_003_t spl07_003;
static struct spl07_003_t *p_spl07_003;

void spl07_003_write(uint8 hwadr, uint8 regadr, uint8 val);
uint8 spl07_003_read(uint8 hwadr, uint8 regadr);
void spl07_003_get_calib_param(void);

/*****************************************************************************
 Function: spl07_003_write
 Description: this function will write data to specofic register through software I2C bus
 Input:  uint8 hwadr   hardware I2C address
         uint8 regadr  register address
         uint8 val     write-in value          
 Output: 
 Return: 
 Calls: 
 Called By: 
*****************************************************************************/
void spl07_003_write(uint8 hwadr, uint8 regadr, uint8 val)
{
    SoftI2C_start();
    SoftI2C_write_byte(hwadr << 1);
    SoftI2C_write_byte(regadr);
    SoftI2C_write_byte(val);
    SoftI2C_stop();
}

/*****************************************************************************
 Function: spl07_003_read
 Description: this function will read register data through software I2C bus
 Input: uint8 hwadr   hardware I2C address
        uint8 regadr  register address        
 Output: 
 Return: uint8 readout value
 Calls: 
 Called By: 
*****************************************************************************/
uint8 spl07_003_read(uint8 hwadr, uint8 regadr)
{
    uint8 val = 0;
    SoftI2C_start();
    SoftI2C_write_byte(hwadr << 1);
    SoftI2C_write_byte(regadr);
    SoftI2C_start();
    SoftI2C_write_byte((hwadr << 1)|0x01);
    val = SoftI2C_read_byte(1);
    SoftI2C_stop();
    return val;
}

void read_chip_id(void)
{
	p_spl07_003->chip_id =	spl07_003_read(HW_ADR, 0x0D);
	printf("check chip id =0x%x\n",p_spl07_003->chip_id);
}

uint8 is_spl07_003(void)
{ 
	if(p_spl07_003->chip_id == 0x11)
		return 1;
	else
		return 0;			
}

/*****************************************************************************
 Function: spl07_003_init
 Description: initialization
 Input: void             
 Output: 
 Return: void 
 Calls: 
 Called By: 
*****************************************************************************/
void spl07_003_init(void)
{
    p_spl07_003 = &spl07_003; /* read Chip Id */
    p_spl07_003->i32rawPressure = 0;
    p_spl07_003->i32rawTemperature = 0;
   // p_spl07_003->chip_id = 0x11;
	read_chip_id();

    spl07_003_get_calib_param();
    // sampling rate = 1Hz; Pressure oversample = 2;
    spl07_003_rateset(PRESSURE_SENSOR,32, 8);   
    // sampling rate = 1Hz; Temperature oversample = 1; 
    spl07_003_rateset(TEMPERATURE_SENSOR,32, 8);
    //Start background measurement
    
}

/*****************************************************************************
 Function: spl07_003_rateset
 Description: set sample rate and over sample rate per second for specific sensor
 Input:     uint8 u8OverSmpl  oversample rate         Maximal = 128
            uint8 u8SmplRate  sample rate(Hz) Maximal = 128
            uint8 iSensor     0: Pressure; 1: Temperature 
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl07_003_rateset(uint8 iSensor, uint8 u8SmplRate, uint8 u8OverSmpl)
{
    uint8 reg = 0;
    int32 i32kPkT = 0;
    switch(u8SmplRate)
    {
        case 2:
            reg |= (1<<4);
            break;
        case 4:
            reg |= (2<<4);
            break;
        case 8:
            reg |= (3<<4);
            break;
        case 16:
            reg |= (4<<4);
            break;
        case 32:
            reg |= (5<<4);
            break;
        case 64:
            reg |= (6<<4);
            break;
        case 128:
            reg |= (7<<4);
            break;
        case 1:
        default:
            break;
    }
    switch(u8OverSmpl)
    {
        case 2:
            reg |= 1;
            i32kPkT = 1572864;
            break;
        case 4:
            reg |= 2;
            i32kPkT = 3670016;
            break;
        case 8:
            reg |= 3;
            i32kPkT = 7864320;
            break;
        case 16:
            i32kPkT = 253952;
            reg |= 4;
            break;
        case 32:
            i32kPkT = 516096;
            reg |= 5;
            break;
        case 64:
            i32kPkT = 1040384;
            reg |= 6;
            break;
        case 128:
            i32kPkT = 2088960;
            reg |= 7;
            break;
        case 1:
        default:
            i32kPkT = 524288;
            break;
    }

    if(iSensor == PRESSURE_SENSOR)
    {
        p_spl07_003->i32kP = i32kPkT;
        spl07_003_write(HW_ADR, 0x06, reg);
        if(u8OverSmpl > 8)
        {
            reg = spl07_003_read(HW_ADR, 0x09);
            spl07_003_write(HW_ADR, 0x09, reg | 0x04);
        }
        else
        {
            reg = spl07_003_read(HW_ADR, 0x09);
            spl07_003_write(HW_ADR, 0x09, reg & (~0x04));
        }
    }
    if(iSensor == TEMPERATURE_SENSOR)
    {
        p_spl07_003->i32kT = i32kPkT;
		if(is_spl07_003())
        	spl07_003_write(HW_ADR, 0x07, reg); 
		else
			spl07_003_write(HW_ADR, 0x07, reg|0x80);  //Using mems temperature	
        if(u8OverSmpl > 8)
        {
            reg = spl07_003_read(HW_ADR, 0x09);
            spl07_003_write(HW_ADR, 0x09, reg | 0x08);
        }
        else
        {
            reg = spl07_003_read(HW_ADR, 0x09);
            spl07_003_write(HW_ADR, 0x09, reg & (~0x08));
        }
    }

}

/*****************************************************************************
 Function: spl07_003_get_calib_param
 Description: obtain the calibrated coefficient
 Input: void     
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl07_003_get_calib_param(void)
{
    uint8 h;
    uint8 m;
    uint8 l;
    h =  spl07_003_read(HW_ADR, 0x10);
    l  =  spl07_003_read(HW_ADR, 0x11);
    p_spl07_003->calib_param.c0 = (int16)h<<4 | l>>4;
    p_spl07_003->calib_param.c0 = (p_spl07_003->calib_param.c0&0x0800)?(0xF000|p_spl07_003->calib_param.c0):p_spl07_003->calib_param.c0;
    h =  spl07_003_read(HW_ADR, 0x11);
    l  =  spl07_003_read(HW_ADR, 0x12);
    p_spl07_003->calib_param.c1 = (int16)(h&0x0F)<<8 | l;
    p_spl07_003->calib_param.c1 = (p_spl07_003->calib_param.c1&0x0800)?(0xF000|p_spl07_003->calib_param.c1):p_spl07_003->calib_param.c1;
    h =  spl07_003_read(HW_ADR, 0x13);
    m =  spl07_003_read(HW_ADR, 0x14);
    l =  spl07_003_read(HW_ADR, 0x15);
    p_spl07_003->calib_param.c00 = (int32)h<<12 | (int32)m<<4 | (int32)l>>4;
    p_spl07_003->calib_param.c00 = (p_spl07_003->calib_param.c00&0x080000)?(0xFFF00000|p_spl07_003->calib_param.c00):p_spl07_003->calib_param.c00;
    h =  spl07_003_read(HW_ADR, 0x15);
    m =  spl07_003_read(HW_ADR, 0x16);
    l =  spl07_003_read(HW_ADR, 0x17);
    p_spl07_003->calib_param.c10 = (int32)(h&0x0F)<<16 | (int32)m<<8 | l;
    p_spl07_003->calib_param.c10 = (p_spl07_003->calib_param.c10&0x080000)?(0xFFF00000|p_spl07_003->calib_param.c10):p_spl07_003->calib_param.c10;
    h =  spl07_003_read(HW_ADR, 0x18);
    l  =  spl07_003_read(HW_ADR, 0x19);
    p_spl07_003->calib_param.c01 = (int16)h<<8 | l;
    h =  spl07_003_read(HW_ADR, 0x1A);
    l  =  spl07_003_read(HW_ADR, 0x1B);
    p_spl07_003->calib_param.c11 = (int16)h<<8 | l;
    h =  spl07_003_read(HW_ADR, 0x1C);
    l  =  spl07_003_read(HW_ADR, 0x1D);
    p_spl07_003->calib_param.c20 = (int16)h<<8 | l;
    h =  spl07_003_read(HW_ADR, 0x1E);
    l  =  spl07_003_read(HW_ADR, 0x1F);
    p_spl07_003->calib_param.c21 = (int16)h<<8 | l;
    h =  spl07_003_read(HW_ADR, 0x20);
    l  =  spl07_003_read(HW_ADR, 0x21);
    p_spl07_003->calib_param.c30 = (int16)h<<8 | l;
	if(is_spl07_003())
	{
	    h = spl07_003_read(HW_ADR, 0x22);
	    l = spl07_003_read(HW_ADR, 0x23);
	    p_spl07_003->calib_param.c31 = (int16)h << 4 | l >> 4;
	    p_spl07_003->calib_param.c31 = (p_spl07_003->calib_param.c31 & 0x0800) ? (0xF000 | p_spl07_003->calib_param.c31) : p_spl07_003->calib_param.c31;
	    h = spl07_003_read(HW_ADR, 0x23);
	    l = spl07_003_read(HW_ADR, 0x24);
	    p_spl07_003->calib_param.c40 = (int16)(h & 0x0F) << 8 | l;
	    p_spl07_003->calib_param.c40 = (p_spl07_003->calib_param.c40 & 0x0800) ? (0xF000 | p_spl07_003->calib_param.c40) : p_spl07_003->calib_param.c40;
	}
}

/*****************************************************************************
 Function: spl07_003_start_temperature
 Description: start one measurement for temperature
 Input: void    
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl07_003_start_temperature(void)
{
    spl07_003_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
 Function: spl07_003_start_pressure
 Description: start one measurement for pressure
 Input: void       
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/

void spl07_003_start_pressure(void)
{
    spl07_003_write(HW_ADR, 0x08, 0x01);
}
/*****************************************************************************
 Function: spl07_003_start_continuous
 Description: Select mode for the continuously measurement
 Input: uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature        
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl07_003_start_continuous(uint8 mode)
{
    spl07_003_write(HW_ADR, 0x08, mode+4);
}

void spl07_003_stop(void)
{
    spl07_003_write(HW_ADR, 0x08, 0);
}

/*****************************************************************************
 Function: spl07_003_get_raw_temp
 Description:obtain the original temperature value and turn them into 32bits-integer 
 Input: void          
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl07_003_get_raw_temp(void)
{
    uint8 h,m,l;
    SoftI2C_start();
    SoftI2C_write_byte(HW_ADR << 1);
    SoftI2C_write_byte(0x03);
    SoftI2C_start();
    SoftI2C_write_byte((HW_ADR << 1)|0x01);
    h = SoftI2C_read_byte(0);
    m = SoftI2C_read_byte(0);
    l = SoftI2C_read_byte(1);
    SoftI2C_stop();
    p_spl07_003->i32rawTemperature = (int32)h<<16 | (int32)m<<8 | (int32)l;
    p_spl07_003->i32rawTemperature= (p_spl07_003->i32rawTemperature&0x800000) ? (0xFF000000|p_spl07_003->i32rawTemperature) : p_spl07_003->i32rawTemperature;
}

/*****************************************************************************
 Function: spl07_003_get_raw_pressure
 Description: obtain the original pressure value and turn them into 32bits-integer
 Input: void       
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
void spl07_003_get_raw_pressure(void)
{
    uint8 h,m,l;
    SoftI2C_start();
    SoftI2C_write_byte(HW_ADR << 1);
    SoftI2C_write_byte(0x00);
    SoftI2C_start();
    SoftI2C_write_byte((HW_ADR << 1)|0x01);
    h = SoftI2C_read_byte(0);
    m = SoftI2C_read_byte(0);
    l = SoftI2C_read_byte(1);
    SoftI2C_stop();
    
    p_spl07_003->i32rawPressure = (int32)h<<16 | (int32)m<<8 | (int32)l;
    p_spl07_003->i32rawPressure= (p_spl07_003->i32rawPressure&0x800000) ? (0xFF000000|p_spl07_003->i32rawPressure) : p_spl07_003->i32rawPressure;
}

/*****************************************************************************
 Function: spl07_003_get_temperature
 Description:  return calibrated temperature value base on original value.
 Input: void          
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/
float spl07_003_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl07_003->i32rawTemperature / (float)p_spl07_003->i32kT;
    fTCompensate =  p_spl07_003->calib_param.c0 * 0.5 + p_spl07_003->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 Function: spl07_003_get_pressure
 Description: return calibrated pressure value base on original value.
 Input: void            
 Output: 
 Return: void
 Calls: 
 Called By: 
*****************************************************************************/

float spl07_003_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl07_003->i32rawTemperature / (float)p_spl07_003->i32kT;
    fPsc = p_spl07_003->i32rawPressure / (float)p_spl07_003->i32kP;
    qua2 = p_spl07_003->calib_param.c10 + fPsc * (p_spl07_003->calib_param.c20 + fPsc* (p_spl07_003->calib_param.c30+ fPsc * p_spl07_003->calib_param.c40));
    if(is_spl07_003())
		qua3 = fTsc * fPsc * (p_spl07_003->calib_param.c11 + fPsc * (p_spl07_003->calib_param.c21+ fPsc * p_spl07_003->calib_param.c31));
	else
		qua3 = fTsc * fPsc * (p_spl07_003->calib_param.c11 + fPsc * p_spl07_003->calib_param.c21);
    fPCompensate = p_spl07_003->calib_param.c00 + fPsc * qua2 + fTsc * p_spl07_003->calib_param.c01 + qua3;
    return fPCompensate;
}




#endif
