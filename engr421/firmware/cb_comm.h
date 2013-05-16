#ifndef COMM_H
#define COMM_H

#include <ch.h>
#include <hal.h>
#include <cb_config.h>

void setup_comm(void);

/* Command structures. TODO: Incorporate this into the actual communications
 * code instead of mapping bytes to floats. */
typedef struct {
	float death_ray_intensity;   /* Hopper motor duty cycle */
	float linear_rail_pos;
} robot_t;

typedef struct {
	bool new_command;
	robot_t one;
	robot_t two;
} command_t;

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
void rxend(UARTDriver *uartp);

/*
 * Collect received bytes and try to extract valid commands.
 */
command_t packet_to_command(uint8_t *buffer, uint16_t bufsize);

/*
 * USART3 configuration structure.
 */
static const UARTConfig uart3cfg = {
	NULL,          /* End of Transmission buffer callback               */
	NULL,          /* Physical end of transmission callback             */
	rxend,         /* Receive buffer filled callback                    */
	NULL,          /* Char received while out of the UART_RECEIVE state */
	NULL,          /* Receive error callback                            */
	460800,        /* Baudrate                                          */
	0,             /* cr1 register values                               */
	0,             /* cr2 register values                               */
	0              /* cr3 register values                               */
};

#endif // COMM_H

