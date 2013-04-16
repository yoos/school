#ifndef COMM_H
#define COMM_H

#include <ch.h>
#include <hal.h>
#include <cb_config.h>

void setup_comm(void);

/*
 * USART3 configuration structure.
 */
static const UARTConfig uart3cfg = {
	NULL,          /* End of Transmission buffer callback               */
	NULL,          /* Physical end of transmission callback             */
	NULL,          /* Receive buffer filled callback                    */
	NULL,          /* Char received while out of the UART_RECEIVE state */
	NULL,          /* Receive error callback                            */
	460800,        /* Baudrate                                          */
	0,             /* cr1 register values                               */
	0,             /* cr2 register values                               */
	0              /* cr3 register values                               */
};

#endif // COMM_H

