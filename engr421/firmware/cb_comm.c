#include <cb_comm.h>

static uint8_t comm_input[COMM_PACKET_LENGTH];   /* Possibly a valid command. */
static uint8_t cmd_buffer[COMM_RECEIVE_BUFFER_LENGTH];   /* Circular buffer to store received bytes. */
static uint16_t b_index;   /* Index to track head of circular buffer. */

void rxend(UARTDriver *uartp)
{
	(void)uartp;

	/* Update commands from serial input. */
	//rc.one.death_ray_intensity = 
}

command_t packet_to_command(uint8_t *buffer, uint16_t bufsize)
{
	uint16_t i;

	command_t rc;   /* Robot commands */
	rc.new_command = false;

	for (i=0; i<bufsize; i++) {
		cmd_buffer[b_index] = buffer[i];

		if (cmd_buffer[b_index] == COMM_HEADER) {   /* Receive header byte. */
			uint8_t j, b;
			for (j=0; j<COMM_PACKET_LENGTH; j++) {   /* Are these valid values? */
				b = cmd_buffer[(COMM_RECEIVE_BUFFER_LENGTH + b_index - COMM_PACKET_LENGTH + j) % COMM_RECEIVE_BUFFER_LENGTH];
				if (b <= COMM_INPUT_MAX) {
					comm_input[j] = b;
					rc.new_command = true;
					palSetPad(GPIOD, 15);
				}
				else {
					rc.new_command = false;
					j = COMM_PACKET_LENGTH;   /* Exit loop. */
				}
			}
		}

		b_index = (b_index+1) % COMM_RECEIVE_BUFFER_LENGTH;   /* TODO: This doesn't look correct. */

		if (rc.new_command) {
			rc.one.death_ray_intensity = ((float) comm_input[0]) / ((float) COMM_INPUT_MAX);
			rc.one.linear_rail_pos     = ((float) comm_input[1]) / ((float) COMM_INPUT_MAX);
			rc.two.death_ray_intensity = ((float) comm_input[2]) / ((float) COMM_INPUT_MAX);
			rc.two.linear_rail_pos     = ((float) comm_input[3]) / ((float) COMM_INPUT_MAX);

			return rc;
		}
	}

	return rc;
}


void setup_comm(void)
{
	/*
	 * Activate USART3.
	 */
	uartStart(&UARTD3, &uart3cfg);

	palSetPadMode(GPIOB, 10, PAL_MODE_ALTERNATE(7));   // USART3 TX
	palSetPadMode(GPIOB, 11, PAL_MODE_ALTERNATE(7));   // USART3 RX

	uint16_t i;
	for (i=0; i<COMM_RECEIVE_BUFFER_LENGTH; i++) {
		cmd_buffer[i] = 0;
	}
}

