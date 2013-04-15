#include <cb_comm.h>

void setup_comm(void)
{
	/*
	 * Activate USART1 and USART3
	 * TODO: I would enable USART2, as well, but for some reason it doesn't
	 * work when I2C1 is enabled.
	 */
	uartStart(&UARTD1, &uart1cfg);
	uartStart(&UARTD3, &uart3cfg);

	palSetPadMode(GPIOB, 6, PAL_MODE_ALTERNATE(7));   // USART1 TX
	palSetPadMode(GPIOB, 7, PAL_MODE_ALTERNATE(7));   // USART1 RX
	palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));   // USART3 TX
	palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));   // USART3 RX
}

