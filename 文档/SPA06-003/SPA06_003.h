#ifndef SPL07_003_H
#define SPL07_003_H
#include "hal_board_cfg.h"

#define HW_ADR 0x77
#define CONTINUOUS_PRESSURE     1
#define CONTINUOUS_TEMPERATURE  2
#define CONTINUOUS_P_AND_T      3
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1


struct spl07_003_calib_param_t {	
    int16 c0;
    int16 c1;
    int32 c00;
    int32 c10;
    int16 c01;
    int16 c11;
    int16 c20;
    int16 c21;
    int16 c30;
    int16 c31;
    int16 c40;
};

struct spl07_003_t {	
    struct spl07_003_calib_param_t calib_param;/**<calibration data*/	
    uint8 chip_id; /**<chip id*/	
    int32 i32rawPressure;
    int32 i32rawTemperature;
    int32 i32kP;    
    int32 i32kT;
};

void spl07_003_init(void);
void spl07_003_rateset(uint8 iSensor, uint8 u8OverSmpl, uint8 u8SmplRate);
void spl07_003_start_temperature(void);
void spl07_003_start_pressure(void);
void spl07_003_start_continuous(uint8 mode);
void spl07_003_get_raw_temp(void);
void spl07_003_get_raw_pressure(void);
float spl07_003_get_temperature(void);
float spl07_003_get_pressure(void);

#endif

