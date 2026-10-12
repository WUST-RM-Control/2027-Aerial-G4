#ifndef __Flash__
#define __Flash__

#include "main.h"
typedef struct
{
    float IMU_OFFSET_Data[5];

} Config_StructTypedef;
extern Config_StructTypedef Config;

void Config_Save(void);
void Config_Read(void);
void Flash_Init(void);

#endif
