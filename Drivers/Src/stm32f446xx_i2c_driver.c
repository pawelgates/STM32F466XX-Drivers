

#include "stm32f446xx_i2c_driver.h"


uint16_t AHB1_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint8_t APB1_PreScaler[4] = {2, 4, 8, 16};

/* Internal functions */

uint32_t RCC_GetPLLOutputClk(void)
{
	uint32_t pll_clk = 0;

	// PLL clock in not used in this project

	return pll_clk;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, system_clk;
	uint16_t ahb1_pre;
	uint8_t clk_src, temp, apb1_pre;

	/* System Clock source */
	clk_src = ((RCC->CFGR >> 2) & 0x3);
	if(clk_src == 0)
	{
		system_clk = 16000000; // HSI Clock
	}
	else if(clk_src == 1)
	{
		system_clk = 8000000; // HSE Clock
	}
	else if(clk_src == 2)
	{
		system_clk = RCC_GetPLLOutputClk(); // PLL Clock
	}

	/* AHB pre-scaler */
	temp = ((RCC->CFGR >> 4) & 0xF);
	if(temp < 8)
	{
		ahb1_pre = 1;
	}
	else
	{
		ahb1_pre = AHB1_PreScaler[temp - 8];
	}

	/* APB1 pre-scaler */
	temp = ((RCC->CFGR >> 10) & 0x7);
	if(temp < 4)
	{
		apb1_pre = 1;
	}
	else
	{
		apb1_pre = APB1_PreScaler[temp - 4];
	}

	pclk1 = (system_clk / ahb1_pre) / apb1_pre;

	return pclk1;
}

static void I2C_GenerateStartConditions(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	/* Shifting address 1 bit to the left */
	SlaveAddr = SlaveAddr << 1;
	/* Clearing LSB bit for write purpose (0) */
	SlaveAddr &= ~(0x1);
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	/* Shifting address 1 bit to the left */
	SlaveAddr = SlaveAddr << 1;
	/* Clearing LSB bit for read purpose (1) */
	SlaveAddr |= 0x1;
	pI2Cx->DR = SlaveAddr;
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->SR1;
	dummyRead = pI2Cx->SR2;
	(void)dummyRead;
}

static void I2C_GenerateStopConditions(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == I2C_ACK_ENABLE)
	{
		// Enable the ACK
		pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK);
	}else
	{
		// Disable the ACK
		pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK);
	}
}


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
	uint8_t tempreg = 0;

	/* Enable peripheral clock control */
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	/* ACK control bit */
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	/* FREQ field of CR2 */
	tempreg = 0;
	tempreg |= (RCC_GetPCLK1Value() / 1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	/* Device own address */
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	/* CCR calculations */
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* Standard mode */
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		/* Fast mode */
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		else
		{
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	/* TRISE configuration */
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		/* Standard mode */
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else
	{
		/* Fast mode */
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000U) + 1;
	}

	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
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
 * @function 				- I2C_GetFlagStatus
 * @brief					- This function returns the flag status (1 or 0)
 *
 * @parameter[in]			- Base address to I2C peripheral
 * @parameter[in]			- Flag name according to status register (SR)
 *
 * @return					- Flag status (0 or 1)
 *
 * @note					- NONE
 */

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}


/*********************************************************************************
 * @function 				- I2C_MasterSendData
 * @brief					- In this function Master sends data to Slave.
 *
 * @parameter[in]			- Address to I2C Handle function
 * @parameter[in]			- Address to TxBuffer
 * @parameter[in]			- Length of TxBuffer
 * @parameter[in]			- Slave address
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	/* Generate start condition */
	I2C_GenerateStartConditions(pI2CHandle->pI2Cx);

	/* Confirm that start generation is completed by checking the SB flag in the SR1 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	/* Send the address of the slave with WRITE bit on [0] position (total 8 bits) */
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	/* 4. Confirm that address phase is completed by checking the ADDR flag in the SR1 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	/* Clear the ADDR flag according to its software sequence */
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	/* Send data until the LEN becomes 0 */
	while(Len < 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG));	// Data register empty
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG));	// Byte transfer finished

	/* Generate stop condition */
	I2C_GenerateStopConditions(pI2CHandle->pI2Cx);

}


/*********************************************************************************
 * @function 				- I2C_MasterReceiveData
 * @brief					- In this function Master receives data from Slave.
 *
 * @parameter[in]			- Address to I2C Handle function
 * @parameter[in]			- Address to RxBuffer
 * @parameter[in]			- Length of RxBuffer
 * @parameter[in]			- Slave address
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr)
{
	/* Generate start condition */
	I2C_GenerateStartConditions(pI2CHandle->pI2Cx);

	/* Confirm that start generation is completed by checking the SB flag in the SR1 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG));

	/* Send the address of the slave with READ bit on [0] position (total 8 bits) */
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	/* Confirm that address phase is completed by checking the ADDR flag in the SR1 */
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	if(Len == 1)
	{
		/* Disable the ACK */
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		/* Clear the ADDR flag according to its software sequence */
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		/* Wait until RXNE flag will be 1 */
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

		/* Generate stop condition */
		I2C_GenerateStopConditions(pI2CHandle->pI2Cx);

		/* Read the data in to buffer */
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if(Len > 1)
	{
		/* Clear the ADDR flag according to its software sequence */
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		for(uint32_t i = Len; i > 0; i--)
		{
			/* Wait until RXNE flag will be 1 */
			while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG));

			if(i == 2)
			{
				/* Disable the ACK */
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				/* Generate stop condition */
				I2C_GenerateStopConditions(pI2CHandle->pI2Cx);
			}

			/* Read the data in to buffer */
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			/* Increment the buffer address */
			pRxBuffer++;
		}
	}

	/* Re-enable the ACK */
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}


/*********************************************************************************
 * @function 				- I2C_PeripheralControl
 * @brief					- This function ENABLEs or DISABLEs the I2Cx peripheral via CR1 register
 *
 * @parameter[in]			- Base address to I2C peripheral
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
