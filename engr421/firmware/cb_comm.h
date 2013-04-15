#ifndef CB_COMM_H
#define CB_COMM_H

#include <ch.h>
#include <hal.h>
#include <cb_config.h>

void setup_comm(void);

/*
 * USART1 configuration structure.
 */
static const UARTConfig uart1cfg = {
	NULL,          /* End of Transmission buffer callback               */
	NULL,          /* Physical end of transmission callback             */
	NULL,          /* Receive buffer filled callback                    */
	NULL,          /* Char received while out of the UART_RECEIVE state */
	NULL,          /* Receive error callback                            */
	38400,         /* Baudrate                                          */
	0,             /* cr1 register values                               */
	USART_CR2_LINEN,   /* cr2 register values */
	0              /* cr3 register values                               */
};

#endif // CB_COMM_H

