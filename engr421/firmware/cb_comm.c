#include <cb_comm.h>

void setup_comm(void)
{
	/*
	 * Activate USART1
	 */
	uartStart(&UARTD1, &uart1cfg);

	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));   // USART1 TX
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(1));   // USART1 RX
}

