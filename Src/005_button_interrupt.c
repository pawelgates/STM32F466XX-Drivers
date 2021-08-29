

#include "stm32f446xx.h"
#include <string.h>

#define BUTTON_PRESSED 		0

void delay(int num)
{
	for(uint32_t i = 0; i < num; i++);
}

void EXTI15_10_IRQHandler(void)
{
	delay(100000);
	GPIO_IRQHandling(GPIO_PIN_NUM_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NUM_5);
}

int main(void)
{
	// LED2 - PA5 pin
	// BUTTON1 - PC13 pin

	GPIO_Handle_t gpio_led, gpio_button;
	memset(&gpio_led, 0, sizeof(gpio_led));
	memset(&gpio_button, 0, sizeof(gpio_button));

	// GPIO LED initialization
	gpio_led.pGPIOX = GPIOA;
	gpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_5;
	gpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OUT_PP;
	gpio_led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpio_led);

	// GPIO BUTTON initialization
	gpio_button.pGPIOX = GPIOC;
	gpio_button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	gpio_button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INT_FT;
	gpio_button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpio_button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&gpio_button);

	//IRQ configuration
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);


	while(1)
	{
		// LOOP
	}


	return 0;
}




