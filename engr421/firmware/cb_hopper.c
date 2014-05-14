#include <cb_hopper.h>

static uint16_t counter;
const uint16_t COUNTER_MAX = ((uint16_t) (1.0/HOPPER_DT/HOPPER_PULSE_FREQUENCY));

void setup_hopper(void)
{
	counter = 0;
}

void update_hopper(uint8_t status, float *dc)
{
	/* Controller */
	if (status == DISABLED || status == STANDBY) {
		/* Disabled or standing by */
		counter = 0;
		*dc = HOPPER_DC_MIN;
	}
	else {
		/* Beating Daniel Miller */

		/* Control */
		if (counter < HOPPER_PULSE_DC * COUNTER_MAX) {
			*dc = HOPPER_DC_MAX;
		}
		else {
			*dc = HOPPER_DC_MIN;
		}

		counter = (counter+1) % COUNTER_MAX;
		
	}
}

void hopper_debug_output(uint8_t *buffer)
{
	(void) buffer;
}

