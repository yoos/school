#include <cb_digital.h>

void setup_digital(uint8_t *state)
{
	/* Inputs */
	palSetPadMode(GPIOD, I_DIGITAL_LR_SWITCH, PAL_MODE_INPUT_PULLUP);   /* Linear rail limit switch */
	palSetPadMode(GPIOD, I_DIGITAL_ENABLE, PAL_MODE_INPUT_PULLUP);   /* Enable */
	palSetPadMode(GPIOD, I_DIGITAL_ARBITER, PAL_MODE_INPUT_PULLUP);   /* Arbiter */

	/* Outputs */
	palSetPadMode(GPIOD, I_DIGITAL_LINEAR_RAIL, PAL_MODE_OUTPUT_PUSHPULL);

	/* Set default desired states */
	state[I_DIGITAL_LR_SWITCH] = 1;
	state[I_DIGITAL_ENABLE] = 1;
	state[I_DIGITAL_ARBITER] = 1;
	state[I_DIGITAL_LINEAR_RAIL] = 0;
}

void update_digital(uint8_t *state)
{
	/* Inputs */
	state[I_DIGITAL_LR_SWITCH] = palReadPad(GPIOD, I_DIGITAL_LR_SWITCH);
	state[I_DIGITAL_ENABLE] = palReadPad(GPIOD, I_DIGITAL_ENABLE);
	state[I_DIGITAL_ARBITER] = palReadPad(GPIOD, I_DIGITAL_ARBITER);

	/* Outputs */
	palWritePad(GPIOD, I_DIGITAL_LINEAR_RAIL, state[I_DIGITAL_LINEAR_RAIL]);
}

