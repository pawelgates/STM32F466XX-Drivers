

#include "stm32f446xx_gpio_driver.h"


/************************ APIs SUPPORTED BY THIS DRIVER **************************/

/*********************************************************************************
 * @function 				- GPIO_PeriClockControl
 * @brief					- This function Enables or Disables peripheral clock for the given GPIO port
 *
 * @parameter[in]			- Base address of the GPIO port
 * @parameter[in]			- ENABLE or DISABLE macros
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_DI();
		}
	}
}


/*********************************************************************************
 * @function 				- GPIO_Init
 * @brief					- This function initialize the GPIO port (Pin / Speed / Pull Up-Down resistors / Output type / Alternate functionality)
 *
 * @parameter[in]			- Base address of the GPIO port
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;

	/* GPIO mode configuration */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOX->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing 2 bits
		pGPIOHandle->pGPIOX->MODER |= temp;	// Setting 2 bits
		temp = 0;
	}
	else
	{
		// Interrupt modes
	}

	/* GPIO speed configuration */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->OSPEEDER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing 2 bits
	pGPIOHandle->pGPIOX->OSPEEDER |= temp;	// Setting 2 bits
	temp = 0;

	/* GPIO pull up/down resistors configuration */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Clearing 2 bits
	pGPIOHandle->pGPIOX->PUPDR |= temp;	// Setting 2 bits
	temp = 0;

	/* GPIO output type configuration */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing 1 bit
	pGPIOHandle->pGPIOX->OTYPER |= temp; // Setting 1 bit
	temp = 0;

	/* GPIO alternate functionality configuration */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOX->AFR[temp1] &= ~(0xF << (4 * temp2)); // Clearing 4 bits
		pGPIOHandle->pGPIOX->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // Setting 4 bits
	}
}


/*********************************************************************************
 * @function 				- GPIO_DeInit
 * @brief					- This function resets the GPIO port (Pin / Speed / Pull Up-Down resistors / Output type / Alternate functionality)
 *
 * @parameter[in]			- Base address of the GPIO port
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_REG_RESET();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}
}


/*********************************************************************************
 * @function 				- GPIO_ReadFromInputPin
 * @brief					- This function reads pin value from GPIO port.
 *
 * @parameter[in]			- Base address of the GPIO port
 * @parameter[in]			- Pin number (0 to 15)
 *
 * @return					- Value of the pin (1 or 0)
 *
 * @note					- NONE
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*********************************************************************************
 * @function 				- GPIO_ReadFromInputPort
 * @brief					- This function reads GPIO port value from all 16 pins.
 *
 * @parameter[in]			- Base address of the GPIO port
 *
 * @return					- Value of the GPIO port (16 bits)
 *
 * @note					- NONE
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t) pGPIOx->IDR;
	return value;
}


/*********************************************************************************
 * @function 				- GPIO_WriteToOutputPin
 * @brief					- This function writes value (1 or 0) to specific pin of GPIO port.
 *
 * @parameter[in]			- Base address of the GPIO port
 * @parameter[in]			- Pin number (0 to 15)
 * @parameter[in]			- Value (1 or 0)
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*********************************************************************************
 * @function 				- GPIO_WriteToOutputPort
 * @brief					- This function writes value (16 bits) to GPIO port.
 *
 * @parameter[in]			- Base address of the GPIO port
 * @parameter[in]			- Value (16 bits)
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*********************************************************************************
 * @function 				- GPIO_ToggleOutputPin
 * @brief					- This function toggles pin value of the GPIO port (1 to 0 or 0 to 1).
 *
 * @parameter[in]			- Base address of the GPIO port
 * @parameter[in]			- Pin number (0 to 15)
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/* Interrupt configuration */

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDi)
{

}

void GPIO_IRQHandling(uint8_t PinNumber)
{

}
