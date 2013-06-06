#include <cb_hopper.h>

static uint16_t counter;
const uint16_t COUNTER_MAX = ((uint16_t) (1.0/HOPPER_DT/HOPPER_PULSE_FREQUENCY));

void setup_hopper(void)
{
	counter = 0;
}

void update_hopper(uint8_t status, float *dc)
{
	uint8_t i;

	/* Controller */
	if (status == DISABLED || status == STANDBY) {
		/* Disabled or standing by */
		for (i=0; i<2; i++) {
			counter = 0;
			dc[i] = HOPPER_DC_MIN;
		}
	}
	else {
		/* Beating Daniel Miller */
		for (i=0; i<2; i++) {
			/* Control */
			if (counter < HOPPER_PULSE_DC * COUNTER_MAX) {
				dc[i] = HOPPER_DC_MAX;
			}
			else {
				dc[i] = HOPPER_DC_MIN;
			}

			counter = (counter+1) % COUNTER_MAX;
		}
	}
}

void hopper_debug_output(uint8_t *buffer)
{
}

