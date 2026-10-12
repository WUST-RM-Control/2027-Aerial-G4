//=====================================================================================
//                    USART_Driver[USART通信驱动]
//      包含了一些USART通信相关的封装好的函数
//
//      USART初始化 USART_Init(void)
//      USART发送 USART1/3/6_Printf(char *format, ...)
//      USART接收事件回调函数统一处理串口收到的数据
//=====================================================================================

#include "USART_Driver.h"

uint8_t UART1_RX_Data[128];
uint8_t UART2_RX_Data[128];
uint8_t UART3_RX_Data[128];

void Remote_Rx_CallBack(uint8_t *data);

/**
 * @brief USART接收空闲事件回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart == &huart1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
    else if(huart == &huart2)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
    else if(huart == &huart3)
	{
        Remote_Rx_CallBack(UART3_RX_Data);
        
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
    if(huart == &huart1)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
    else if(huart == &huart2)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
	}
    else if(huart == &huart3)
	{
        
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RX_Data, 128);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	}
}

/**
 * @brief 串口初始化
 *
 * @param 无
 */
void USART_Init(void)
{    
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1_RX_Data, 128);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, UART2_RX_Data, 128);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, UART3_RX_Data, 128);
    
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
}

