#include <cb_comm.h>

static command_t rc;   /* Robot commands */

void rxend(UARTDriver *uartp)
{
	(void)uartp;

	/* Update commands from serial input. */
	//rc.one.death_ray_intensity = 
}


void setup_comm(void)
{
	/*
	 * Activate USART3.
	 */
	uartStart(&UARTD3, &uart3cfg);

	palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));   // USART3 TX
	palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));   // USART3 RX
}

