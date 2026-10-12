#include "LED.h"
#include "Sensor.h"

#define LED_TIM 

uint16_t LED_ARR = 20;
uint16_t LED_CNT = 0;
uint16_t LED_CCR_R = 0;
uint16_t LED_CCR_G = 0;
uint16_t LED_CCR_B = 0;


//10ms
void LED_Task(void const * argument)
{
    while(1)
    {
        LED_CNT += 1;
        if(LED_CNT >= LED_ARR) LED_CNT = 0;
        
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, (LED_CNT < LED_CCR_R));
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, (LED_CNT < LED_CCR_G));
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, (LED_CNT < LED_CCR_B));
        
//        LED_Floow_Step();  
        //8.4v-6.6V
        int16_t H = (Sensor.BatV-6.6f)/1.8f * 360.0f;
        LED_Set_H(H);
        
        osDelay(1);
    }
}

void LED_ON(LED_Color Color, float Light)
{
	switch((uint8_t)Color)
	{
		case 1: LED_CCR_R = Light * LED_ARR; break;
		case 2: LED_CCR_G = Light * LED_ARR; break;
		case 3: LED_CCR_B = Light * LED_ARR; break;
	}
}

void LED_OFF(LED_Color Color)
{
	switch((uint8_t)Color)
	{
		case 1: LED_CCR_R = 0;break;
		case 2: LED_CCR_G = 0;break;
		case 3: LED_CCR_B = 0;break;
	}
}

/**
 * @brief HSV转RGB
 *
 * @param 色环角度
 * @param 明度
 * @param 饱和度
 */
uint8_t R_Color,G_Color,B_Color;
void HSVtoRGB(uint16_t h, uint16_t s, uint16_t v)      
{
    if(h >= 360) h = 360;

    if(s >= 100) s = 100;

    if(v >= 100) v = 100;

    int i;
    R_Color = 0;
    B_Color = 0;
    G_Color = 0;
    i = h / 60;
    int difs = h % 60; 
    float RGB_max = v * 2.55f;
    float RGB_min = RGB_max * (100 - s) / 100.0f;
    float RGB_Adj = (RGB_max - RGB_min) * difs / 60.0f; 
    switch(i)
    {
    case 0:
        R_Color = RGB_max;
        G_Color = RGB_min + RGB_Adj;
        B_Color = RGB_min;
        break;

    case 1:
        R_Color = RGB_max - RGB_Adj;
        G_Color = RGB_max;
        B_Color = RGB_min;
        break;

    case 2:
        R_Color = RGB_min;
        G_Color = RGB_max;
        B_Color = RGB_min + RGB_Adj;
        break;

    case 3:
        R_Color = RGB_min;
        G_Color = RGB_max - RGB_Adj;
        B_Color = RGB_max;
        break;

    case 4:
        R_Color = RGB_min + RGB_Adj;
        G_Color = RGB_min;
        B_Color = RGB_max;
        break;

    default:
        R_Color = RGB_max;
        G_Color = RGB_min;
        B_Color = RGB_max - RGB_Adj;
        break;
    }
}


void LED_Floow_Step(void)
{
    static float H_BASE = 0;
    HSVtoRGB(H_BASE,100,100);
	H_BASE = fmodf((H_BASE + 0.1), 360);

    LED_ON(LED_R, R_Color / 256.0f);
    LED_ON(LED_G, G_Color / 256.0f);
    LED_ON(LED_B, B_Color / 256.0f);
}

void LED_Set_H(int16_t H)
{
    if(H < 0) H = 0;
    HSVtoRGB(H,100,100);

    LED_ON(LED_R, R_Color / 256.0f);
    LED_ON(LED_G, G_Color / 256.0f);
    LED_ON(LED_B, B_Color / 256.0f);
        
}

