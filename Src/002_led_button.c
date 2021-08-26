

#include "stm32f446xx.h"

#define BUTTON_PRESSED 		0

void delay(int num)
{
	for(uint32_t i = 0; i < num; i++);
}

int main(void)
{
	// LED2 - PA5 pin
	// BUTTON1 - PC13 pin

	GPIO_Handle_t gpio_led, gpio_button;

	// GPIO LED initialization
	gpio_led.pGPIOX = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// GPIO BUTTON initialization
	gpio_button.pGPIOX = GPIOC;
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpio_led);
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpio_button);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NUM_13) == BUTTON_PRESSED)
		{
			delay(250000);
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
		}
	}


	return 0;
}
