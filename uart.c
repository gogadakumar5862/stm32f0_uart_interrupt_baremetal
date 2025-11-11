#include "uart.h"

#define UART2EN     (1U<<17)

#define CR1_TE      (1U<<3)
#define CR1_RE      (1U<<2)
#define CR1_UE      (1U<<0)

#define SR_TXE      (1U<<7)
#define SR_RXNE      (1U<<5)

#define CR1_RXNEIE    (1U<<5)


#define SYS_FREQ    8000000
#define APB1_CLK    SYS_FREQ

#define UART_BAUDRATE  115200

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periphClk, uint32_t baud);
static uint16_t compute_uart_bd(uint32_t periphClk, uint32_t baud);


// ================================================================
 void uart2_tx_init(void);  //  -----> use only for polling method
// ===================================================================


void uart2_write(uint8_t ch);

void uart2_rxtx_interrupt_init(void)
{
	/*********Configure uart gpio pin ******/
	/* Enable clock access to gpioa */

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Set PA2 mode to alternate function mode */
		GPIOA->MODER &=~(1U<<4);
		GPIOA->MODER |= (1U<<5);

	/* Set PA2 alternate function type to UART_TX (AF1) */
		GPIOA->AFR[0] |= (1U<<8);
		GPIOA->AFR[0] &=~(1U<<9);
		GPIOA->AFR[0] &=~(1U<<10);
		GPIOA->AFR[0] &=~(1U<<11);

	/* Set PA3 mode to alternate function mode */
				GPIOA->MODER &=~(1U<<6);
				GPIOA->MODER |= (1U<<7);

	/* Set PA3 alternate function type to UART_TX (AF1) */
				GPIOA->AFR[0] |= (1U<<12);
				GPIOA->AFR[0] &=~(1U<<13);
				GPIOA->AFR[0] &=~(1U<<14);
				GPIOA->AFR[0] &=~(1U<<15);


	/****** Configure UART module ********/
		/* Enable clock access to uart2 */
		RCC->APB1ENR |= UART2EN;

	/* Configure baudrate  */
		uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);

	/* Configure uart transfer direction  */
		USART2->CR1 = (CR1_TE | CR1_RE);

	/* Enable the RXNE interrupt */
		USART2->CR1 |=CR1_RXNEIE;

	/* Enable uart2 interrupt in the NVIC */
		NVIC_EnableIRQ(USART2_IRQn);

	/* Enable UART module */
		USART2->CR1 |= CR1_UE;

}


char uart2_read(void)
{
    // Wait until RXNE flag is set
    while (!(USART2->ISR & (1U << 5)));   // RXNE = bit 5
    return (char)(USART2->RDR & 0xFF);    // Read data
}

void uart2_write(uint8_t ch)
{
	/* Make sure transmit data register is empty */
		while(!(USART2->ISR & SR_TXE)){}
    // Wait until TXE flag is set

    USART2->TDR = (ch & 0xFF);

    // Wait until TC flag is set
    while (!(USART2->ISR & (1U << 6)));   // TC = bit 6
}


static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t periphClk, uint32_t baud)
{
	USARTx->BRR = compute_uart_bd(periphClk, baud);
}

static uint16_t compute_uart_bd(uint32_t periphClk, uint32_t baud)
{
	return (periphClk + (baud / 2U)) / baud;
}

//===================================================================================================================================================================================================================
//   Use the uart2_tx_init only when polling method is used 
void uart2_tx_init(void)
{
	/*********Configure uart gpio pin ******/
	/* Enable clock access to gpioa */

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Set PA2 mode to alternate function mode */
		GPIOA->MODER &=~(1U<<4);
		GPIOA->MODER |= (1U<<5);

	/* Set PA2 alternate function type to UART_TX (AF1) */
		GPIOA->AFR[0] |= (1U<<8);
		GPIOA->AFR[0] &=~(1U<<9);
		GPIOA->AFR[0] &=~(1U<<10);
		GPIOA->AFR[0] &=~(1U<<11);

	/****** Configure UART module ********/
		/* Enable clock access to uart2 */
		RCC->APB1ENR |= UART2EN;

	/* Configure baudrate  */
		uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE);

	/* Configure uart transfer direction  */
		USART2->CR1 = CR1_TE;

	/* Enable UART module */
		USART2->CR1 |= CR1_UE;

}

// ===========================================================================================================================================
