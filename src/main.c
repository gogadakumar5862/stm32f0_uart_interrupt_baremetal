#include <stdint.h>
#include "stm32f0xx.h"
#include "uart.h"



char c;
int main()
{
	uart2_rxtx_interrupt_init();
	while(1)
	{
		//c=uart2_read(); // wait until a char is received
		//uart2_write(c+1);
	}
}

void USART2_IRQHandler(void)
{
	if(USART2->ISR & (1U<<5))
	{
		c=uart2_read(); // wait until a char is received
		uart2_write(c+1);
	}
}
