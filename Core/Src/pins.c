
// Includes---------------------------------------------------------------------------------------------//
#include "pins.h"
#include "typedef.h"

//--------------------------------------------Configure GPIO--------------------------------------------//
void Pins_LEDs_Init(void)
{
/*	uint8_t count;
	
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  // GPIO Ports Clock Enable 
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

	const TPortPin pins[] = 
	{
		{ LED_RED_PORT, LED_RED_PIN },
		{ LED_GREEN_PORT, LED_GREEN_PIN },
	};
	
	//инициализация пинов с выходами на светодиоды
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT; //режим на выход
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW; //скорость низкая
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL; //режим - пушпулл

	for(count = 0 ; count < sizeof( pins )/sizeof( pins[0] ) ; count++ )
	{
		
		GPIO_InitStruct.Pin = pins[count].Pin; // номер пина
		LL_GPIO_Init( pins[count].PORTx, &GPIO_InitStruct); //инициализация структуры
	}*/

}

//-----------------------------------------------------------------------------------------------------//
void Pins_Address_Init(void)
{
	uint8_t count;
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	const TPortPin pins[] = {
		{ MY_BACKPLANE_ADDR0_PORT, MY_BACKPLANE_ADDR0_PIN },
		{ MY_BACKPLANE_ADDR1_PORT, MY_BACKPLANE_ADDR1_PIN },
		{ MY_BACKPLANE_ADDR2_PORT, MY_BACKPLANE_ADDR2_PIN },
		{ MY_BACKPLANE_ADDR3_PORT, MY_BACKPLANE_ADDR3_PIN },
		{ MY_BACKPLANE_ADDR4_PORT, MY_BACKPLANE_ADDR4_PIN }
	};
	
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);

	GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT; //режим - вход
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW; //скорость низкая
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP; //подтяжка вверх

	for( count = 0 ; count < sizeof( pins )/sizeof( pins[0] ) ; count++ )
	{
		
		GPIO_InitStruct.Pin = pins[count].Pin; // номер пина
		LL_GPIO_Init( pins[count].PORTx, &GPIO_InitStruct); //инициализация GPIO
	}

	HAL_Delay(400); //delay
}

//-----------------------------------------------------------------------------------------------------//
uint8_t Get_Module_Address( void )
{
  uint8_t count;
 // uint32_t result = 0;
	uint8_t addr = 0x3;
	
/*	Pins_Address_Init();
	
  const TPortPin pins[] = 
						{
							{ MY_BACKPLANE_ADDR0_PORT, MY_BACKPLANE_ADDR0_PIN },
							{ MY_BACKPLANE_ADDR1_PORT, MY_BACKPLANE_ADDR1_PIN },
              { MY_BACKPLANE_ADDR2_PORT, MY_BACKPLANE_ADDR2_PIN },
							{ MY_BACKPLANE_ADDR3_PORT, MY_BACKPLANE_ADDR3_PIN },
							{ MY_BACKPLANE_ADDR4_PORT, MY_BACKPLANE_ADDR4_PIN }
						};

	for( count = 0 ; count < 5; count++ ) //cчитывание состояния пинов на кросс плате
	{
		addr |= (!(LL_GPIO_IsInputPinSet (pins[count].PORTx, pins[count].Pin)) << count); //инверсия
	}*/

	return addr;
}

//-----------------------------------------------------------------------------------------------------//
void Task_Control_LEDs( void )
{
/*	if( g_MyFlags.CAN_Fail == 1 ) //если на CAN шине ошибка
	{
		LED_GREEN(OFF); //красный включён, зелёный выключен
		LED_RED(ON); 
	}
	else 
	{
		LED_RED(OFF);
		LED_GREEN(ON);
	}*/
}

//-----------------------------------------------------------------------------------------------------//
