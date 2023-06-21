/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "pins.h"
#include "MNP_msg.h"
#include "usart.h"
#include "typedef.h"
#include <stdio.h>
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
MNP_MSG_t MNP_PUT_MSG; //иницализация шаблона сообщения для отправки приёмнику
MNP_MSG_t MNP_GET_MSG;

osTimerId osProgTimerGPSUARTTimeout;  
osTimerId osProgTimerGPSCfg;

osThreadId Task_Parse_GPS_msg_Handle;
osThreadId Task_Switch_Led_Handle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void osProgTimerGPSUARTTimeoutCallback(void const *argument);
void osProgTimerGPSCfgCallback (void const *argument);

void Parse_GPS_msg (void const * argument);
void Switch_Led (void const * argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

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
  osTimerDef (osTimerGPSUARTTimeout, osProgTimerGPSUARTTimeoutCallback);
	osProgTimerGPSUARTTimeout = osTimerCreate(osTimer (osTimerGPSUARTTimeout), osTimerPeriodic, NULL);
	
	osTimerDef (osTimerGPSCfg, osProgTimerGPSCfgCallback);
	osProgTimerGPSCfg = osTimerCreate(osTimer (osTimerGPSCfg), osTimerOnce, NULL);
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	osThreadDef (Task_Parse_GPS_msg, Parse_GPS_msg, osPriorityNormal, 0, 256); 
	Task_Parse_GPS_msg_Handle = osThreadCreate(osThread(Task_Parse_GPS_msg), NULL); 	
	
  osThreadDef (Task_Switch_Led, Switch_Led, osPriorityLow, 0, 128); 
	Task_Switch_Led_Handle = osThreadCreate(osThread(Task_Switch_Led), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	GPS_Init(&MNP_PUT_MSG);
	osDelay (250);
//	read_config_MNP (&MNP_PUT_MSG);
//	osDelay (250);
  /* Infinite loop */
  for(;;)
  {
	//	read_config_MNP (&MNP_PUT_MSG);
	//	put_msg2000 (&MNP_PUT_MSG);
    osDelay(5000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//--------------------------------------------парсинг gps-сообщения--------------------------------------------//
void Parse_GPS_msg (void const * argument)
{
	int8_t result = -1;
	uint16_t byte_i = 0;
	osTimerStart(osProgTimerGPSUARTTimeout, 5000);
	MNP_GET_MSG.rx_state = 1;
	
	for(;;)
  {
		result = (Parse_MNP_MSG (&MNP_GET_MSG));
		
/*		if (result < 0)
		{
			osTimerStart(osProgTimerGPSUARTTimeout, 5000);
		}*/
		
		if (MNP_GET_MSG.rx_state != 1)
		{
			sprintf (buffer_TX_UART2, "return byte_i=%u, state=%u\r\n", byte_i, MNP_GET_MSG.rx_state);
			UART2_PutString (buffer_TX_UART2);
		}
		
		
		osDelay (100);
	}
}

//---------------------------------------------мигание светодиодом---------------------------------------------//
void Switch_Led (void const * argument)
{
	for(;;)
  {
		TOOGLE_LED_RED();
		osDelay (1000);
	}
}

//------------------------------------------------------------------------------------------//
void osProgTimerGPSUARTTimeoutCallback(void const *argument)
{
	MNP_Reset(&MNP_PUT_MSG);
	osTimerStart(osProgTimerGPSCfg, GPS_RST_DELAY); //задержка перед отправкой конф. сообщения приёмнику
}

//------------------------------------------------------------------------------------------//
void osProgTimerGPSCfgCallback (void const *argument)
{
//	GPS_Init(&MNP_PUT_MSG);
}

/* USER CODE END Application */

