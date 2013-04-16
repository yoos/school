#include <cb_comm.h>

void setup_comm(void)
{
	/*
	 * Activate USART3.
	 */
	uartStart(&UARTD3, &uart3cfg);

	palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));   // USART3 TX
	palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));   // USART3 RX
}

