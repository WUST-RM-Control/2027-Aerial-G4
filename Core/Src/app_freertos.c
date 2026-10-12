/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId INSHandle;
osThreadId ControlHandle;
osThreadId SensorHandle;
osThreadId RemoteHandle;
osThreadId BuzzerHandle;
osThreadId LEDHandle;
osThreadId CommunicateHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void INS_Task(void const * argument);
void Control_Task(void const * argument);
void Sensor_Task(void const * argument);
void Remote_Task(void const * argument);
void Buzzer_Task(void const * argument);
void LED_Task(void const * argument);
void Communicate_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of INS */
  osThreadDef(INS, INS_Task, osPriorityAboveNormal, 0, 4096);
  INSHandle = osThreadCreate(osThread(INS), NULL);

  /* definition and creation of Control */
  osThreadDef(Control, Control_Task, osPriorityHigh, 0, 1024);
  ControlHandle = osThreadCreate(osThread(Control), NULL);

  /* definition and creation of Sensor */
  osThreadDef(Sensor, Sensor_Task, osPriorityAboveNormal, 0, 1024);
  SensorHandle = osThreadCreate(osThread(Sensor), NULL);

  /* definition and creation of Remote */
  osThreadDef(Remote, Remote_Task, osPriorityAboveNormal, 0, 512);
  RemoteHandle = osThreadCreate(osThread(Remote), NULL);

  /* definition and creation of Buzzer */
  osThreadDef(Buzzer, Buzzer_Task, osPriorityIdle, 0, 512);
  BuzzerHandle = osThreadCreate(osThread(Buzzer), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, LED_Task, osPriorityIdle, 0, 256);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of Communicate */
  osThreadDef(Communicate, Communicate_Task, osPriorityLow, 0, 512);
  CommunicateHandle = osThreadCreate(osThread(Communicate), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* init code for USB_Device */
  MX_USB_Device_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_Sensor_Task */
/**
* @brief Function implementing the Sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Sensor_Task */
__weak void Sensor_Task(void const * argument)
{
  /* USER CODE BEGIN Sensor_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Sensor_Task */
}

/* USER CODE BEGIN Header_Remote_Task */
/**
* @brief Function implementing the Remote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Remote_Task */
__weak void Remote_Task(void const * argument)
{
  /* USER CODE BEGIN Remote_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Remote_Task */
}

/* USER CODE BEGIN Header_Buzzer_Task */
/**
* @brief Function implementing the Buzzer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Buzzer_Task */
__weak void Buzzer_Task(void const * argument)
{
  /* USER CODE BEGIN Buzzer_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Buzzer_Task */
}

/* USER CODE BEGIN Header_LED_Task */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_Task */
__weak void LED_Task(void const * argument)
{
  /* USER CODE BEGIN LED_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_Task */
}

/* USER CODE BEGIN Header_Communicate_Task */
/**
* @brief Function implementing the Communicate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Communicate_Task */
__weak void Communicate_Task(void const * argument)
{
  /* USER CODE BEGIN Communicate_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Communicate_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

