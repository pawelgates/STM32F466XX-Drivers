

#include "stm32f446xx_spi_driver.h"

/************************ APIs SUPPORTED BY THIS DRIVER **************************/
/*********************************************************************************/


/*********************************************************************************
 * @function 				- SPI_PeriClockControl
 * @brief					- This function Enables or Disables peripheral clock for the given SPI peripheral
 *
 * @parameter[in]			- Base address of SPI
 * @parameter[in]			- ENABLE or DISABLE
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_CLK_EN();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_CLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_CLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_CLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_CLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_CLK_DI();
		}
	}
}


/*********************************************************************************
 * @function 				- SPI_Init
 * @brief					- This function initialize the SPI peripheral
 *
 * @parameter[in]			- Address to SPI Handle function
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/* Enable the peripheral clock */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* SPI_CR1 configuration */
	uint32_t temp = 0;

	/* Device mode */
	temp |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	/* Bus configuration */
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_FULLDUPLEX)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_HALFDUPLEX)
	{
		temp |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUSCONFIG_SIMPLEX_RX)
	{
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		temp |= (1 << SPI_CR1_RXONLY);
	}

	/* SCLK speed configuration */
	temp |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	/* DFF configuration */
	temp |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	/* CPOL configuration */
	temp |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	/* CPHA configuration */
	temp |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandle->pSPIx->CR1 = temp;

}

/*********************************************************************************
 * @function 				- SPI_DeInit
 * @brief					- This function resets the SPI peripheral via RCC Bus Reset register
 *
 * @parameter[in]			- Base address to SPI peripheral
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}


/*********************************************************************************
 * @function 				- SPI_PeripheralControl
 * @brief					- This function ENABLEs or DISABLEs the SPIx peripheral
 *
 * @parameter[in]			- Base address to SPI peripheral
 * @parameter[in]			- ENABLE or DISABLE
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}


/*********************************************************************************
 * @function 				- SPI_SSIConfig
 * @brief					- This function ENABLEs or DISABLEs the SPIx peripheral SSI bit
 *
 * @parameter[in]			- Base address to SPI peripheral
 * @parameter[in]			- ENABLE or DISABLE
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*********************************************************************************
 * @function 				- SPI_SSOEConfig
 * @brief					- This function ENABLEs or DISABLEs the SPIx peripheral SSOE bit
 *
 * @parameter[in]			- Base address to SPI peripheral
 * @parameter[in]			- ENABLE or DISABLE
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*********************************************************************************
 * @function 				- SPI_GetFlagStatus
 * @brief					- This function returns the flag status (1 or 0)
 *
 * @parameter[in]			- Base address to SPI peripheral
 * @parameter[in]			- Flag name according to status register (SR)
 *
 * @return					- Flag status (0 or 1)
 *
 * @note					- NONE
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************************
 * @function 				- SPI_SendData
 * @brief					- This function will send data from TxBuffer via SPIx
 *
 * @parameter[in]			- Base address to SPI peripheral
 * @parameter[in]			- Pointer to TxBuffer
 * @parameter[in]			- Length of the buffer in bytes
 *
 * @return					- NONE
 *
 * @note					- This is blocking call
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* Waiting until TXE flag is set */
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		/* Checking the DFF in CR1 register */
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			/* 16 bit data frame format */
			/* Load data from RX buffer to DR */
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			/* 8 bit data frame format */
			/* Load data from RX buffer to DR */
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}


/*********************************************************************************
 * @function 				- SPI_ReceiveData
 * @brief					-
 *
 * @parameter[in]			-
 * @parameter[in]			-
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* Waiting until RXNE flag is set */
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		/* Checking the DFF in CR1 register */
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			/* 16 bit data frame format */
			/* Load data from DR to RX buffer */
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t *)pRxBuffer++;
		}
		else
		{
			/* 8 bit data frame format */
			/* Load data from DR to RX buffer */
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}

	}
}


/*********************************************************************************
 * @function 				- SPI_IRQInterruptConfig
 * @brief					-
 *
 * @parameter[in]			-
 * @parameter[in]			-
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{

}


/*********************************************************************************
 * @function 				- SPI_IRQPriorityConfig
 * @brief					-
 *
 * @parameter[in]			-
 * @parameter[in]			-
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}


/*********************************************************************************
 * @function 				- SPI_IRQHandling
 * @brief					-
 *
 * @parameter[in]			-
 * @parameter[in]			-
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

