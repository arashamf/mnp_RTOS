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
#include "protocol.h"
#include <stdio.h>
#include <string.h>
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
MNP_MSG_t MNP_GET_MSG =
{
	.rx_state = __SYNC_BYTE1 //иницализация структуры сообщения полученного от приёмника
};

MNP_M7_CFG_t MNP_M7_CFG = //иницализация шаблона структуры перезагрузки и настройки приёмника
{
	
	.cfg_state = __SYNC_RST ,
	.parse_delay = GPS_PARSE_DELAY,
	.cfg_msg_delay = GPS_CFG_MSG_DELAY,
	.rst_delay = GPS_RST_DELAY,	//задержка для перезагрузки приёмника
};

osTimerId osProgTimerGPSUARTTimeout;  
osTimerId osProgTimerGPSCfg;
osTimerId osProgTimerGPSReset;

osThreadId Task_Parse_GPS_msg_Handle;
osThreadId Task_Switch_Led_Handle;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void osProgTimerGPSUARTTimeoutCallback(void const *argument);
void osProgTimerGPSCfgCallback (void const *argument);
void osProgTimerGPSResetCallback (void const *argument);

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
	
	osTimerDef (osTimerGPSReset, osProgTimerGPSResetCallback);
	osProgTimerGPSReset = osTimerCreate(osTimer (osTimerGPSReset), osTimerOnce, NULL);
	
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
	osThreadDef (Task_Parse_GPS_msg, Parse_GPS_msg, osPriorityAboveNormal, 0, 256); 
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
//	osDelay (100);
	GPS_Init(&MNP_PUT_MSG);
	osDelay (100);
	Set_GNSS_interval (&MNP_PUT_MSG, 2000); //2000=1c
	osDelay (100);
	
  for(;;)
  {	
		//Get_GNSS_interval (&MNP_PUT_MSG, 10000); //2000=1c, 1000=5c
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
	uint32_t tickcount = osKernelSysTick();
	osTimerStart(osProgTimerGPSUARTTimeout, 5000); //таймера обработки таймаута получения сообщения от приёмника
	
	for(;;)
  {
	//	result = (GPS_wait_data_Callback (&MNP_GET_MSG)); //парсинг сообщения от приёмника		
		if ((result = (GPS_wait_data_Callback (&MNP_GET_MSG))) > 0) //если сообщение от приёмника получено успешно
		{
			osTimerStart(osProgTimerGPSUARTTimeout, 5000); //перегрузка таймера обработки таймаута
			if (MKS2.tmContext.ValidTHRESHOLD == 0) //если сообщение получено, но данные недостоверны
			{
				MNP_M7_CFG.parse_delay = 1000;
				osTimerStart(osProgTimerGPSReset, 50); //запуск таймера перезагрузки модуля				
			}
		}
		osDelayUntil (&tickcount, MNP_M7_CFG.parse_delay);
	}
}

//---------------------------------------------мигание светодиодом---------------------------------------------//
void Switch_Led (void const * argument)
{
	LED_RED(ON);
	for(;;)
  {
		TOOGLE_LED_RED();
		osDelay (1000);
	}
}

//------------------------------------------------------------------------------------------//
void osProgTimerGPSUARTTimeoutCallback(void const *argument)
{
		sprintf (buffer_TX_UART2, "gps_timeout\r\n");
		UART2_PutString (buffer_TX_UART2);
		osTimerStart(osProgTimerGPSReset, 50); //запуск таймера перезагрузки модуля
	//MNP_Reset(&MNP_PUT_MSG); //отправка команды перезагрузки приёмника
	//osTimerStart(osProgTimerGPSCfg, GPS_RST_DELAY); //задержка перед отправкой конф. сообщения приёмнику
}

//------------------------------------------------------------------------------------------//
void osProgTimerGPSCfgCallback (void const *argument)
{
	switch(MNP_M7_CFG.cfg_state) 
	{
		case __SYNC_RST: //если стадия аппартной перезагрузки модуля
			//GPS_rst(DISABLE);
			MNP_M7_CFG.cfg_state = __SYNC_LOAD_CFG;
			osTimerStart(osProgTimerGPSCfg, MNP_M7_CFG.cfg_msg_delay);
			break;
		
		case __SYNC_LOAD_CFG: //стадия отправки конф. сообщений приёмнику
			GPS_Init(&MNP_PUT_MSG);
			MNP_M7_CFG.parse_delay = GPS_PARSE_DELAY;
			break;
		
		default:
			break;
	}
}

//------------------------------------------------------------------------------------------//
void osProgTimerGPSResetCallback (void const *argument)
{
	MNP_Reset(&MNP_PUT_MSG);
	//GPS_rst(ENABLE); //аппаратная перезагрузка модуля
	MNP_M7_CFG.cfg_state = __SYNC_RST ;
	osTimerStart(osProgTimerGPSCfg, MNP_M7_CFG.rst_delay); //запуск таймера отправки конфигурационных сообщений модулю через 500мс
}
/* USER CODE END Application */

