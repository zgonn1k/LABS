
#include "main.h"
#include "stm32f4xx_it.h"

void SysTick_Handler(void)
{
	us_ticks++;
}
