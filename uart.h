
#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "stm32f0xx.h"

void uart2_rxtx_interrupt_init();
void uart2_rxtx_init(void);
void uart2_write(int ch);
char uart2_read(void);


#endif /* UART_H_ */
