

#include "stm32f446xx_i2c_driver.h"


/************************ APIs SUPPORTED BY THIS DRIVER **************************/
/*********************************************************************************/

/*********************************************************************************
 * @function 				- I2C_PeriClockControl
 * @brief					- This function Enables or Disables peripheral clock for the given I2C peripheral
 *
 * @parameter[in]			- Base address of I2C
 * @parameter[in]			- ENABLE or DISABLE
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_CLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_CLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_CLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_CLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_CLK_DI();
		}
	}
}


/*********************************************************************************
 * @function 				- I2C_Init
 * @brief					- This function initialize the I2C peripheral
 *
 * @parameter[in]			- Address to I2C Handle function
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void I2C_Init(I2C_Handle_t *pI2CHandle)
{

}


/*********************************************************************************
 * @function 				- I2C_DeInit
 * @brief					- This function resets the I2C peripheral via RCC Bus Reset register
 *
 * @parameter[in]			- Base address to I2C peripheral
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}


/*********************************************************************************
 * @function 				- I2C_PeripheralControl
 * @brief					- This function ENABLEs or DISABLEs the I2Cx peripheral via CR1 register
 *
 * @parameter[in]			- Base address to SPI peripheral
 * @parameter[in]			- ENABLE or DISABLE
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}


/*********************************************************************************
 * @function 				- I2C_IRQInterruptConfig
 * @brief					- This function ENABLEs or DISABLEs specific IRQ in NVIC registers.
 *
 * @parameter[in]			- IRQ number
 * @parameter[in]			- ENABLE or DISABLE
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 32 && IRQNumber < 64)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 32 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}


/*********************************************************************************
 * @function 				- I2C_IRQPriorityConfig
 * @brief					- This function sets the priority of specific IRQ in NVIC registers.
 *
 * @parameter[in]			- IRQ number
 * @parameter[in]			- IRQ priority
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_bits = (iprx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_bits);
}
