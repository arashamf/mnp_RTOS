/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */
#pragma anon_unions(1)
#pragma pack(1)

typedef union
{
	struct
	{
		uint8_t data_type		: 3;
		uint8_t module_type	: 5;
		union
		{
			struct
			{
				uint8_t year		: 8;
				uint8_t month		: 8;
				uint8_t day			: 8;
				uint8_t hour		: 8;
				uint8_t min			: 8;
				uint8_t sec			: 8;
				uint8_t local_tz	: 5;
				uint8_t moskow_tz	: 3;
			}Type2;
			
			struct
			{
				uint32_t time2k;
				struct
				{
					int8_t ls_tai					: 8; 	// TAI leap seconds (TAI-UTC)
					uint8_t moscow_tz			: 3;	// в часах
					uint8_t 							: 3;
					uint8_t ls_59					: 1;
					uint8_t ls_61					: 1;				
					int8_t local_tz				: 8;	// в 15 мин интервалах
				};
			}Type3;			
		};
	};

	uint8_t RAW[8];

}MESSAGE_A1_t;

typedef union
{
	struct
	{
	uint8_t data_type			: 3;
	uint8_t module_id			: 5;
	uint8_t 							: 5;
	uint8_t fail_gps_ant 	: 1; 		// 1 - КЗ GPS антены  
	uint8_t fail_gps 			: 1; 		// 1 - отказ GPS приемника
	uint8_t fail 					: 1; 		// 1 - интегральный отказ, при наличие хотябы одного отказа
	uint8_t 							: 7;
	uint8_t gps_ant_disc	: 1;		// 1 - GPS антена неподключенна
	uint8_t 							: 8;
	uint8_t 							: 8;
	uint8_t 							: 8;
	uint8_t 							: 8;
	uint8_t 							: 8;
	};
	uint8_t RAW[8];
} MESSAGE_C2_t;

typedef union
{
	struct
	{
		uint8_t data_type		: 3;
		uint8_t module_type	: 5;
		uint16_t gDOP;
		uint8_t 						: 8;
		uint8_t 						: 8;
		uint8_t 						: 8;
		uint8_t 						: 8;
		uint8_t 						: 8;
	};
	uint8_t RAW[8];
}MESSAGE_B_CONFIG_t;

typedef union
{
	struct
	{
		uint8_t data_type		: 3;
		uint8_t module_type	: 5;
		uint8_t 			 			: 6;
		uint8_t disable			: 1;
		uint8_t reset				: 1;
		uint8_t 						: 8;
		uint8_t 						: 8;
		uint8_t 						: 8;
		uint8_t 						: 8;
		uint8_t 						: 8;
		uint8_t 						: 8;
	};
	uint8_t RAW[8];
}MESSAGE_B_SERVICE_t;

typedef enum { RX_NONE = 0, RX_C1, RX_OWN_C2, RX0, RX1}  TRxResult; //статусы полученных сообщений CAN

#pragma pack()
#pragma anon_unions()
/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
void Task_CANRX (void);
uint32_t Send_Message_C2  (void);
uint32_t Send_Message_A1 (void);
uint32_t CAN1_Send_Message (CAN_TxHeaderTypeDef * TxHeader, uint8_t * CAN_TxData);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

