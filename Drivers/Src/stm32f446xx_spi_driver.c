

#include "stm32f446xx_spi_driver.h"

/* Private functions for Interrupt Handle */

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle);


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
 * @brief					- This function will send data from TxBuffer to DR
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
 * @brief					- This function will receive data from DR and will save it to RX buffer
 *
 * @parameter[in]			- Base address to SPI peripheral
 * @parameter[in]			- Pointer to RxBuffer
 * @parameter[in]			- Length of the buffer in bytes
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
 * @function 				- SPI_SendDataIT
 * @brief					- This function will trigger the SPI IRQ Handler for sending the data.
 *
 * @parameter[in]			- SPI handle function
 * @parameter[in]			- Pointer to TxBuffer
 * @parameter[in]			- Length of the buffer in bytes
 *
 * @return					- State of SPI peripheral
 *
 * @note					- This is a non-blocking call
 */

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		/* Saving the TxBuffer and Len addresses in global variables */
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		/* Marking the busy state */
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		/* Enable the TXEIE control bit */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	return state;
}


/*********************************************************************************
 * @function 				- SPI_ReceiveDataIT
 * @brief					- This function will trigger the SPI IRQ Handler for receiving the data.
 *
 * @parameter[in]			- SPI handle function
 * @parameter[in]			- Pointer to RxBuffer
 * @parameter[in]			- Length of the buffer in bytes
 *
 * @return					- State of SPI peripheral
 *
 * @note					- This is a non-blocking call
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		/* Saving the RxBuffer and Len addresses in global variables */
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;

		/* Marking the busy state */
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		/* Enable the RXNEIE control bit */
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	return state;
}


/*********************************************************************************
 * @function 				- SPI_IRQInterruptConfig
 * @brief					- This function ENABLEs or DISABLEs specific IRQ in NVIC registers.
 *
 * @parameter[in]			- IRQ number
 * @parameter[in]			- ENABLE or DISABLE
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
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
 * @function 				- SPI_IRQPriorityConfig
 * @brief					- This function sets the priority of specific IRQ in NVIC registers.
 *
 * @parameter[in]			- IRQ number
 * @parameter[in]			- IRQ priority
 *
 * @return					- NONE
 *
 * @note					- NONE
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_bits = (iprx_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_bits);
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
	uint8_t temp1, temp2;

	/* Checking TXE */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	/* Checking RXNE */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	/* Checking OVR flag */
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}
}


/* Private functions for Interrupt Handle */

static void spi_txe_interrupt_handle(SPI_Handle_t *pHandle)
{
	/* Checking the DFF in CR1 register */
	if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		/* 16 bit data frame format */
		/* Load data from RX buffer to DR */
		pHandle->pSPIx->DR = *((uint16_t *)pHandle->pTxBuffer);
		pHandle->TxLen--;
		pHandle->TxLen--;
		(uint16_t *)pHandle->pTxBuffer++;
	}
	else
	{
		/* 8 bit data frame format */
		/* Load data from RX buffer to DR */
		pHandle->pSPIx->DR = *pHandle->pTxBuffer;
		pHandle->TxLen--;
		pHandle->pTxBuffer++;
	}

	if(!pHandle->TxLen)
	{
		/* Disable TXE interrupt */
		SPI_CloseTransmission(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pHandle)
{
	/* Checking the DFF in CR1 register */
	if(pHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		/* 16 bit data frame format */
		/* Load data from DR to RX buffer */
		*((uint16_t *)pHandle->pRxBuffer) = pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->RxLen--;
		(uint16_t *)pHandle->pRxBuffer++;
	}
	else
	{
		/* 8 bit data frame format */
		/* Load data from DR to RX buffer */
		*pHandle->pRxBuffer = pHandle->pSPIx->DR;
		pHandle->RxLen--;
		pHandle->pRxBuffer++;
	}

	if(!pHandle->RxLen)
	{
		/* Disable RXNE interrupt */
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle, SPI_EVENT_RX_CMPLT);
	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pHandle)
{
	uint8_t temp;

	/* Clearing the OVR flag */
	if(pHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pHandle->pSPIx->DR;
		temp = pHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pHandle, SPI_EVENT_OVR_ERR);
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	/* Disable TXE interrupt */
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	/* Disable RXNE interrupt */
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	//This is a weak implementation . the user application may override this function.
}

