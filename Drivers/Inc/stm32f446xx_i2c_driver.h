

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_

#include "stm32f446xx.h"

/* Configuration structure for I2Cx peripheral */

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint8_t  I2C_FMDutyCycle;

} I2C_Config_t;

/* Handle structure for I2Cx peripheral */

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

} I2C_Handle_t;


/**************************** MACROS FOR THIS DRIVER *****************************/
/*********************************************************************************/

/* @I2C_SCLSpeed */

#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM2K		200000
#define I2C_SCL_SPEED_FM4K		400000

/* @I2C_DeviceAddress - user configuration*/

/* @I2C_ACKControl */

#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/* @I2C_FMDutyCycle */

#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


/************************ APIs SUPPORTED BY THIS DRIVER **************************/
/*********************************************************************************/

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

uint32_t RCC_GetPLLOutputClk(void);
uint32_t RCC_GetPCLK1Value(void);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
