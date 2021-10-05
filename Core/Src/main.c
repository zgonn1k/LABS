
#include "main.h"

volatile uint32_t us_ticks;

int main(void)
{
	system_core_cfg();
	gpio_init();

	SystemCoreClock = 168000000;
	SysTick_Config(SystemCoreClock / TICK_US);

	while (1)
	{
		set_pin_state(GPIOD, 15, PIN_ON);
		us_delay(1);
		set_pin_state(GPIOD, 15, PIN_OFF);
		us_delay(1);
	}
}



void system_core_cfg(void)
{
	FLASH->ACR = FLASH_ACR_LATENCY_5WS | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;	//AHB = SYSCLK / 1 = 84MHz
	RCC->CFGR |= RCC_CFGR_PPRE1_2;		//APB1 = AHB / 2 = 42MHz, APB1 Timer Clock = APB1 * 2 = 84MHz
	RCC->CR |= RCC_CR_HSEON;
	while ((RCC->CR & RCC_CR_HSERDY) != RCC_CR_HSERDY)
	{

	}

	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2; // 8MHz / 4 = 2MHz
	RCC->PLLCFGR |= (RCC_PLLCFGR_PLLN_3 | RCC_PLLCFGR_PLLN_5 | RCC_PLLCFGR_PLLN_7); // 2MHz * 168 = 336MHz
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP; // 336MHz / 2 = 168MHz -> SYSCLK

	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) != RCC_CR_PLLRDY)
	{

	}

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SW_PLL) != RCC_CFGR_SW_PLL)
	{

	}
}

void gpio_init(void)
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  GPIOD->MODER |= GPIO_MODER_MODE15_0;
  GPIOD->OTYPER &= ~GPIO_OTYPER_OT15;
  GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD15;
  GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR15;
}


/*----------------------------------------------------------------------------
* SysTick_Handler:
*----------------------------------------------------------------------------*/


void us_delay(uint32_t delay_ticks)
{
	uint32_t cur_ticks;
	cur_ticks = us_ticks;
	while ((us_ticks - cur_ticks) < delay_ticks)
	{
		__NOP();
	}
}

void set_pin_state(GPIO_TypeDef *GPIOx, uint8_t pin, uint8_t pin_state)
{
	if (pin_state != PIN_OFF)
	{
		GPIOx->BSRR = (1 << pin);
	}
	else
	{
		GPIOx->BSRR = (1 << pin) << 16;
	}
}

