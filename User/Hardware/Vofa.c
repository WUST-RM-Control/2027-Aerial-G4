#include "Vofa.h"

uint8_t Data_Num = 0;
float Vofa_Data[16];

#define Vofa_Send(Data, Length) HAL_UART_Transmit(&huart1, Data, Length, 1000)

void Vofa_JF_Add_Data(uint8_t Index, float Data)
{
    Data_Num = MAX(Index, Data_Num);
    Vofa_Data[Index] = Data;
    
}

void Vofa_JF_Send(void)
{
    ((uint32_t *)Vofa_Data)[Data_Num+1] = 0x7f800000;
    Vofa_Send((uint8_t *)Vofa_Data, 4*(Data_Num+2));
    Data_Num = 0;
}
