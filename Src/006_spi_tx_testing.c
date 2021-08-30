

#include "stm32f446xx.h"
#include <string.h>

// PB9 - SPI2 NSS pin (AF5)
// PB10 - SPI2 SCK pin (AF5)
// PB14 - SPI2 MISO pin (AF5)
// PB15 - SPI2 MOSI pin (AF5)


void SPI_GPIOInits(void)
{
	GPIO_Handle_t spi2_pins;

	spi2_pins.pGPIOX = GPIOB;
	spi2_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spi2_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	spi2_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_PP;
	spi2_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	spi2_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCK pin
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_10;
	GPIO_Init(&spi2_pins);

	// MOSI pin
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_15;
	GPIO_Init(&spi2_pins);

	// MISO pin
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_14;
	GPIO_Init(&spi2_pins);

	// NSS pin
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_9;
	GPIO_Init(&spi2_pins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t spi2_handle;

	spi2_handle.pSPIx = SPI2;
	spi2_handle.SPIConfig.SPI_BusConfig = SPI_BUSCONFIG_FULLDUPLEX;
	spi2_handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2_handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	spi2_handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	spi2_handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	spi2_handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	spi2_handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&spi2_handle);
}

int main(void)
{
	char user_data[] = "Hello world!";

	/* GPIO initialization for SPI2 */
	SPI_GPIOInits();

	/* SPI2 initialization */
	SPI2_Inits();

	/* SPI2 SSI configuration - NNS signal internally high */
	SPI_SSIConfig(SPI2, ENABLE);

	/* SPI2 enable */
	SPI_PeripheralControl(SPI2, ENABLE);

	/* Sending the data */
	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	/* SPI2 disable */
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
