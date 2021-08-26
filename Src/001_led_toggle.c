

#include "stm32f446xx.h"

void delay(int num)
{
	for(uint32_t i = 0; i < num; i++);
}

int main(void)
{
	// LED2 - PA5 pin
	// BUTTON1 - PC13 pin

	GPIO_Handle_t gpio_led;

	// Port initialization
	gpio_led.pGPIOX = GPIOA;

	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpio_led);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		delay(500000);
	}


	return 0;
}


