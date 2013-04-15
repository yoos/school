#include <cb_motor.h>

void setup_motors()
{
	pwmStart(&PWMD1, &pwm1cfg);
	pwmStart(&PWMD8, &pwm8cfg);

	palSetPadMode(GPIOA, 8,  PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOA, 9,  PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOA, 11, PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOC, 6,  PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOC, 7,  PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOC, 8,  PAL_MODE_ALTERNATE(1));
	palSetPadMode(GPIOC, 9,  PAL_MODE_ALTERNATE(1));
}

void update_motors(float *dc)
{
	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc[0]*10000));
	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc[1]*10000));
	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc[2]*10000));
	pwmEnableChannel(&PWMD1, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD1, dc[3]*10000));
	pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc[4]*10000));
	pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc[5]*10000));
	pwmEnableChannel(&PWMD8, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc[6]*10000));
	pwmEnableChannel(&PWMD8, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc[7]*10000));
}

