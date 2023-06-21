/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdbool.h>
#include <string.h> 
#include "usart.h"
#include "pins.h"
#include "protocol.h"

typedef struct
{
	uint8_t flag_RX;
	uint8_t RxData[8];
}
CAN_RX_msg;

//Macro -----------------------------------------------------------------------------------------------//
#define MAKE_FRAME_ID( msg_type_id, board_addr) ((((uint32_t)msg_type_id) << 5) | (board_addr)) //получение ID заголовка
#define MAKE_MSG_DATA0(__module_id, __data_type) ( ( __module_id << 3 ) | __data_type ) //старшие 5 бит 1 байта сообщения тип модуля-отправителя или тип модуля получателя, младшие 3 бита - код вида данных
#define GET_MODULE_ADDR( frame_id) ((frame_id) & 0x1F) //возвращает адрес модуля (5 бит) из заголовка CAN сообщения
#define GET_MSG_TYPE( frame_id) (((frame_id) >> 5) & 0x3F) //тип сообщения (6 бит) из заголовка CAN сообщения

//Private variables -----------------------------------------------------------------------------------//
static CAN_TxHeaderTypeDef CAN_TxHeader; //структура TxHeader отвечает за отправку кадров
static CAN_RxHeaderTypeDef CAN_RxHeader; //структура для приёма сообщения CAN1

static CAN_RX_msg CAN1_RX;
static uint32_t TxMailbox = 0;//номер почтового ящика для отправки 

static TM_CONTEXT_t 	*tmContext;			// ШВ
static FAIL_CONTEXT_t *fContext;			// отказы
static CAN_CONTEXT_t 	*canContext;		// CAN
static CONFIG_CONTEXT_t *cfgContext; 	// конфигурация

extern MKS2_t MKS2;

static MESSAGE_C2_t MESSAGE_C2; //структура для сообщения типа С2
static MESSAGE_A1_t MESSAGE_A1; //структура для сообщения А1
//static MESSAGE_B_CONFIG_t MESSAGE_B_CONFIG; //структура для получаемого конфигурационного сообщения типа В
//static MESSAGE_B_SERVICE_t MESSAGE_B_SERVICE; //структура для получаемого настроечного сообщения типа В

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//--------------------------------------коллбэк для буфера приёма FIFO №0--------------------------------------//
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO0, &CAN_RxHeader, CAN1_RX.RxData) == HAL_OK) //если пришло прерывание получения пакета в буфер FIFO0 CAN1
	{
		CAN1_RX.flag_RX = RX0; //установка статуса приёма CAN: принято неиндентифицированное сообщение 
		sprintf (buffer_TX_UART2, (char *)"FIFO0_id=%x,msg=%x_%x\r\n", CAN_RxHeader.StdId, CAN1_RX.RxData[0], CAN1_RX.RxData[1]);
		UART2_PutString (buffer_TX_UART2);
	}	
}

//---------------------------------коллбек ошибки по переполнению Fifo0---------------------------------//
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
	g_MyFlags.CAN_Fail = 1;
	sprintf (buffer_TX_UART2, (char *)"CAN_FIFO0_Full");
	UART2_PutString (buffer_TX_UART2);
}

//----------------------------------коллбэк для буфера приёма FIFO №1----------------------------------//
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
	if(HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO1, &CAN_RxHeader, CAN1_RX.RxData) == HAL_OK) //если пришло прерывание получения пакета в буфер FIFO0 CAN1
	{
		CAN1_RX.flag_RX = RX1; //установка статуса приёма CAN: принято неиндентифицированное сообщение 
		sprintf (buffer_TX_UART2, (char *)"FIFO1_id=%x, msg=%x_%x\r\n", CAN_RxHeader.StdId, CAN1_RX.RxData[0], CAN1_RX.RxData[1]);
		UART2_PutString (buffer_TX_UART2);
	}	
}

//---------------------------------коллбек ошибки по переполнению Fifo1---------------------------------//
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan)
{
	g_MyFlags.CAN_Fail = 1;
	sprintf (buffer_TX_UART2, (char *)"CAN_FIFO1_Full");
	UART2_PutString (buffer_TX_UART2);
}

//------------------------------------------коллбек ошибок CAN------------------------------------------//
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t errorcode = 0;
	if ((errorcode = HAL_CAN_GetError(hcan)) != HAL_OK)
	{
		g_MyFlags.CAN_Fail = 1;
		sprintf (buffer_TX_UART2, (char *)"CAN1_ERROR=%u\r\n", errorcode);
		UART2_PutString (buffer_TX_UART2);
	}
}	

//---------------------------Чтение сообщений C1 (от МИУ) и C2 (собственных)---------------------------//
static TRxResult ReadMsgCAN(void)
{
	if (CAN1_RX.flag_RX == RX_NONE)
		{return CAN1_RX.flag_RX;}
	
	if( CAN_RxHeader.RTR == CAN_RTR_REMOTE) //если в полученном сообщении установлен бит RTR
	{
		if ((CAN_RxHeader.StdId & MKS2.canContext.Addr) == MKS2.canContext.Addr)
			{return (CAN1_RX.flag_RX = RX_C1);} //получено сообщение С1
	}
	else
	{
		// Если не установлен RTR - проверка, что это наше собственное сообщение
		if ( (CAN_RxHeader.DLC == 8) && ( CAN_RxHeader.RTR == CAN_RTR_DATA) && ((CAN_RxHeader.StdId & MKS2.canContext.Addr) == MKS2.canContext.Addr))
			{return (CAN1_RX.flag_RX = RX_OWN_C2);}
	}
	return CAN1_RX.flag_RX;
}

//-------------------------------------------------------------------------------------------------------//
void Task_CANRX(void)
{
	static uint32_t errorcode; //код ошибки CAN
  static bool need_init = true;
  static uint32_t last_c2_tx_ticks; //static - значения сохраняются между вызовами 
  uint32_t current_ticks;

	current_ticks = HAL_GetTick();

	if( need_init )
	{
		last_c2_tx_ticks = current_ticks; //сохранения текущего значения времени
		need_init = false;
	}
	
	switch(CAN1_RX.flag_RX)
	{
		case RX_C1: //получен запрос C1  

			/*if ((errorcode = Send_Message_C2()) != HAL_OK ) //отправление сообщения C2
				{g_MyFlags.CAN_Fail = 1;} //если отправка не удалась - установка флага отказа CAN
			else
				{g_MyFlags.CAN_Fail = 0;}  // сброс флага отказа CAN*/
			CAN1_RX.flag_RX = RX_NONE; //сброс флага полученного CAN сообщения
			last_c2_tx_ticks = current_ticks; //запоминание времени отправки сообщения С2
			break;

		case RX_OWN_C2: //получено собственное сообщение C2 

			g_MyFlags.CAN_Fail = 0;  // сброс флага отказа CAN
			//last_c2_rx_ticks = current_ticks; //сохранение текущего количества тиков
			CAN1_RX.flag_RX = RX_NONE; //сброс флага полученного CAN сообщения
			break;

		case RX_NONE:
		break;
		
		case RX0:
			if( CAN_RxHeader.RTR == CAN_RTR_REMOTE) //если в полученном сообщении установлен бит RTR
			{
				if ((CAN_RxHeader.StdId & MKS2.canContext.Addr) == MKS2.canContext.Addr)
				{
					CAN1_RX.flag_RX = RX_C1;  //получено сообщение С1
					break;
				}
			}					
			CAN1_RX.flag_RX = RX_NONE; //сброс флага полученного CAN сообщения
			break;
		
		case RX1:
			CAN1_RX.flag_RX = RX_NONE; //сброс флага полученного CAN сообщения
			break;
		
		default:
		break;
	}
	
	if( current_ticks - last_c2_tx_ticks > 4*TICKS_PER_SECOND ) //если долго (4с) не отправляли С2 (в ответ на С1)
	{		
	/*	if ((errorcode = Send_Message_C2 ()) != HAL_OK ) //отправка сообщения C2 для контроля работоспособности CAN, получение статуса отправки
		{
			g_MyFlags.CAN_Fail = 1; // установка флага отказа CAN
			CAN_Reinit ();	
		} */
		last_c2_tx_ticks = current_ticks; //сохранение текущего количества тиков
	}
}


//------------------------------------------------------------------------------------------------------------//
uint32_t Send_Message_C2 (void)
{
	uint32_t errorcode; //код ошибки CAN
	uint8_t *msg_flags;
	uint8_t CAN_Tx_buffer[8];

	//формирование CAN - заголовка
	CAN_TxHeader.StdId = MAKE_FRAME_ID(MSG_TYPE_C, (uint8_t)canContext->Addr); //ID заголовка сообщения С2
	CAN_TxHeader.ExtId = 0;
	CAN_TxHeader.RTR = CAN_RTR_DATA; //тип сообщения (CAN_RTR_Data - передача данных)
	CAN_TxHeader.IDE = CAN_ID_STD;   //формат кадра Standard
	CAN_TxHeader.DLC = 0x8; //количество байт в сообщении
	CAN_TxHeader.TransmitGlobalTime = 0;
	
	MESSAGE_C2.module_id = canContext->ID;
	MESSAGE_C2.data_type = 0;
	
	if ( (fContext->Fail & FAIL_MASK) != 0 ) 
		{MESSAGE_C2.fail = 1;} 
	else 
		{MESSAGE_C2.fail = 0;}

	MESSAGE_C2.fail_gps = fContext->GPS;
	MESSAGE_C2.fail_gps_ant = fContext->GPSAntShortCircuit;
	MESSAGE_C2.gps_ant_disc = fContext->GPSAntDisconnect;
		
	memcpy(CAN_Tx_buffer, MESSAGE_C2.RAW, 8);		
	CAN1_Send_Message (&CAN_TxHeader, CAN_Tx_buffer);
		
	return errorcode;
}

//------------------------------------------------------------------------------------------------------------//
uint32_t Send_Message_A1 (void)
{
	uint32_t errorcode; //код ошибки CAN
	uint8_t *msg_flags;
	uint8_t CAN_Tx_buffer[8];

	//формирование CAN - заголовка
	CAN_TxHeader.StdId = MAKE_FRAME_ID(MSG_TYPE_A1, (uint8_t)canContext->Addr); //ID заголовка сообщения С2
	CAN_TxHeader.ExtId = 0;
	CAN_TxHeader.RTR = CAN_RTR_DATA; //тип сообщения (CAN_RTR_Data - передача данных)
	CAN_TxHeader.IDE = CAN_ID_STD;   //формат кадра Standard
	CAN_TxHeader.DLC = 0x8; //количество байт в сообщении
	CAN_TxHeader.TransmitGlobalTime = 0;
	
	MESSAGE_C2.module_id = canContext->ID;
	MESSAGE_C2.data_type = 0;
	
	if ( (fContext->Fail & FAIL_MASK) != 0 ) 
		{MESSAGE_C2.fail = 1;} 
	else 
		{MESSAGE_C2.fail = 0;}

	MESSAGE_C2.fail_gps = fContext->GPS;
	MESSAGE_C2.fail_gps_ant = fContext->GPSAntShortCircuit;
	MESSAGE_C2.gps_ant_disc = fContext->GPSAntDisconnect;
		
	memcpy(CAN_Tx_buffer, MESSAGE_C2.RAW, 8);		
	CAN1_Send_Message (&CAN_TxHeader, CAN_Tx_buffer);
		
	return errorcode;
}

//------------------------------------------------------------------------------------------------------------//
uint32_t CAN1_Send_Message (CAN_TxHeaderTypeDef * TxHeader, uint8_t * CAN_TxData)
{
	uint32_t errorcode; //код ошибки CAN
	uint32_t uwCounter = 0;
	
	
	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) && (uwCounter != 0xFFFF)) //ожидание освобождения TxMailbox
		{uwCounter++;} 
	
	if (uwCounter == 0xFFFF)	//выход по тайм-ауту
		{return (errorcode = HAL_TIMEOUT);}
	
	if (READ_BIT (CAN1->TSR, CAN_TSR_TME0)) 
		{TxMailbox = 0;}
	else
	{
		if (READ_BIT (CAN1->TSR, CAN_TSR_TME1)) 
			{TxMailbox = 1;}
		else
		{
			if (READ_BIT (CAN1->TSR, CAN_TSR_TME2)) 
				{TxMailbox = 2;}
		}
	}
	return (errorcode = HAL_CAN_AddTxMessage(&hcan, TxHeader, CAN_TxData, &TxMailbox)); //Добавление сообщений в первый свободный Mailboxe Tx и активация запроса на передачу  
}
/* USER CODE END 1 */
