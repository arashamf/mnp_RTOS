/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PINS_H__
#define __PINS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private defines */

#define ON 1
#define OFF 0

//#define LED_GREEN(x) ((x)? (LL_GPIO_SetOutputPin (LED_GREEN_PORT, LED_GREEN_PIN)) : (LL_GPIO_ResetOutputPin(LED_GREEN_PORT, LED_GREEN_PIN)))
//#define LED_RED(x) ((x)? (LL_GPIO_SetOutputPin (LED_RED_PORT, LED_RED_PIN)) : (LL_GPIO_ResetOutputPin(LED_RED_PORT, LED_RED_PIN)))

#define LED_RED(x) ((x)? (LL_GPIO_SetOutputPin (LED_GPIO_Port, LED_Pin)) : (LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin)))
#define TOOGLE_LED_RED() (LED_RED (!(LL_GPIO_IsOutputPinSet(LED_GPIO_Port, LED_Pin))))

typedef struct 
{
	GPIO_TypeDef *PORTx;
	uint32_t Pin;		
} TPortPin;

/*Prototypes */

void Pins_LEDs_Init(void);
void Pins_Address_Init(void);
uint8_t Get_Module_Address( void );
void Task_Control_LEDs( void );

/*Prototypes*/

#ifdef __cplusplus
}
#endif
#endif /*__ PINS_H__ */

