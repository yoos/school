#include <cb_motor.h>

void setup_motors()
{
	/*
	 * Set up servo pins.
	 */
	pwmStart(&PWMD1, &pwm1cfg);
	pwmStart(&PWMD8, &pwm8cfg);

	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOC, 6, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOC, 7, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOC, 8, PAL_MODE_ALTERNATE(2));
	palSetPadMode(GPIOC, 9, PAL_MODE_ALTERNATE(2));
}

void update_motors(float *dc)
{
	pwmEnableChannel(&PWMD1, 0, PWM_FRACTION_TO_WIDTH(&PWMD1, 1000, dc[0]));
	pwmEnableChannel(&PWMD1, 1, PWM_FRACTION_TO_WIDTH(&PWMD1, 1000, dc[1]));
	pwmEnableChannel(&PWMD1, 2, PWM_FRACTION_TO_WIDTH(&PWMD1, 1000, dc[2]));
	pwmEnableChannel(&PWMD1, 3, PWM_FRACTION_TO_WIDTH(&PWMD1, 1000, dc[3]));
	pwmEnableChannel(&PWMD8, 0, PWM_FRACTION_TO_WIDTH(&PWMD8, 1000, dc[4]));
	pwmEnableChannel(&PWMD8, 1, PWM_FRACTION_TO_WIDTH(&PWMD8, 1000, dc[5]));
	pwmEnableChannel(&PWMD8, 2, PWM_FRACTION_TO_WIDTH(&PWMD8, 1000, dc[6]));
	pwmEnableChannel(&PWMD8, 3, PWM_FRACTION_TO_WIDTH(&PWMD8, 1000, dc[7]));
}

