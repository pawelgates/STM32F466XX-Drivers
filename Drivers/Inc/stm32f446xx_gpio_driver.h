

#ifndef INC_STM32F446XX_GPIO_DRIVER_H_
#define INC_STM32F446XX_GPIO_DRIVER_H_

#include "stm32f446xx.h"

/* Configuration structure for GPIO pin */

typedef struct
{
	uint8_t GPIO_PinNumber;				/* Values from @GPIO_PIN_NUMBER */
	uint8_t GPIO_PinMode;				/* Values from @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;				/* Values from @GPIO_OUTPUT_SPEED */
	uint8_t GPIO_PinPuPdControl;		/* Values from @GPIO_PULL_UP_PULL_DOWN */
	uint8_t GPIO_PinOPType;				/* Values from @GPIO_OUTPUT_TYPES */
	uint8_t GPIO_PinAltFunMode;

} GPIO_PinConfig_t;


/* Handle structure for GPIO pin */

typedef struct
{
	GPIO_RegDef_t *pGPIOX;			/* Pointer to GPIO base address */
	GPIO_PinConfig_t GPIO_PinConfig;

} GPIO_Handle_t;


/* @GPIO_PIN_NUMBER */

#define GPIO_PIN_NUM_0					0
#define GPIO_PIN_NUM_1					1
#define GPIO_PIN_NUM_2					2
#define GPIO_PIN_NUM_3					3
#define GPIO_PIN_NUM_4					4
#define GPIO_PIN_NUM_5					5
#define GPIO_PIN_NUM_6					6
#define GPIO_PIN_NUM_7					7
#define GPIO_PIN_NUM_8					8
#define GPIO_PIN_NUM_9					9
#define GPIO_PIN_NUM_10					10
#define GPIO_PIN_NUM_11					11
#define GPIO_PIN_NUM_12					12
#define GPIO_PIN_NUM_13					13
#define GPIO_PIN_NUM_14					14
#define GPIO_PIN_NUM_15					15


/* @GPIO_PIN_MODES */

#define GPIO_MODE_IN					0
#define GPIO_MODE_OUT					1
#define GPIO_MODE_ALTFN					2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_INT_FT					4
#define GPIO_MODE_INT_RT					5
#define GPIO_MODE_INT_RFT				6



/* @GPIO_OUTPUT_TYPES */

#define GPIO_OUT_PP						0
#define GPIO_OUT_OD						1


/* @GPIO_OUTPUT_SPEED */

#define GPIO_SPEED_LOW					0
#define GPIO_SPEED_MED					1
#define GPIO_SPEED_FAST					2
#define GPIO_SPEED_HIGH					3


/* @GPIO_PULL_UP_PULL_DOWN */

#define GPIO_NO_PUPD					0
#define GPIO_PU							1
#define GPIO_PD							2



/* APIs supported by this driver */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446XX_GPIO_DRIVER_H_ */
