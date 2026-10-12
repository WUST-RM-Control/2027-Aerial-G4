#include <stdio.h>
#include <stdlib.h>
#include "SPL07_003.h"

void Main(void)
{
	float fSplTemp;
	float fSplPressure;
    spl07_003_init();
	// sampling rate = 32Hz; Pressure oversample = 8;
    spl07_003_rateset(PRESSURE_SENSOR,32, 8);   
	// sampling rate = 32Hz; Temperature oversample = 8; 
    spl07_003_rateset(TEMPERATURE_SENSOR,32, 8);
	spl07_003_start_continuous(CONTINUOUS_P_AND_T);
	while(1)
	{
		delay(30ms);
		spl07_003_get_raw_temp();
		spl07_003_get_raw_pressure();
		fSplTemp = spl07_003_get_temperature();
		fSplPressure = spl07_003_get_pressure();
	}
}





