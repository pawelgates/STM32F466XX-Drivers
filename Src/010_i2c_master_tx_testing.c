

#include "stm32f446xx.h"
#include <string.h>

#define MY_ADDR			0x61
#define SLAVE_ADDR		0x68

// PB6 - I2C1 SCL (AF4)
// PB9 - I2C1 SDA (AF4)

I2C_Handle_t i2c1_handle;
uint8_t data[] = "We are testing i2c communication. \n";

void I2C_GPIOInits(void)
{
	GPIO_Handle_t i2c_gpio;
	i2c_gpio.pGPIOX = GPIOB;
	i2c_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_gpio.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_gpio.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_OD;
	i2c_gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	i2c_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL Pin
	i2c_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_6;
	GPIO_Init(&i2c_gpio);

	// SDA Pin
	i2c_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_9;
	GPIO_Init(&i2c_gpio);

}

void I2C1_Inits(void)
{
	i2c1_handle.pI2Cx = I2C1;
	i2c1_handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	i2c1_handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	i2c1_handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	i2c1_handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

}


int main(void)
{
	/* I2C GPIO pin and Peripheral initialization */
	I2C_GPIOInits();
	I2C1_Inits();

	/* I2C Peripheral enable */
	I2C_PeripheralControl(I2C1, ENABLE);

	/* Send the data to the slave */
	I2C_MasterSendData(&i2c1_handle, data, strlen((char*)data), SLAVE_ADDR);

	return 0;
}

