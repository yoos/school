#include <cb_motor.h>

void setup_motors()
{
	pwmStart(&PWMD1, &pwm1cfg);

	palSetPadMode(GPIOE,  9, PAL_MODE_ALTERNATE(1));   /* CH 1 */
	palSetPadMode(GPIOA,  8, PAL_MODE_ALTERNATE(1));   /* CH 2 */
	palSetPadMode(GPIOE, 13, PAL_MODE_ALTERNATE(1));   /* CH 3 */
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(1));   /* CH 4 */

	//pwmStart(&PWMD2, &pwm2cfg);

	//palSetPadMode(GPIOA,  5, PAL_MODE_ALTERNATE(1));   /* CH 1 */
	//palSetPadMode(GPIOA,  1, PAL_MODE_ALTERNATE(1));   /* CH 2 */
	//palSetPadMode(GPIOA,  2, PAL_MODE_ALTERNATE(1));   /* CH 3 */
	//palSetPadMode(GPIOA,  3, PAL_MODE_ALTERNATE(1));   /* CH 4 */

	pwmStart(&PWMD8, &pwm8cfg);

	palSetPadMode(GPIOC,  6, PAL_MODE_ALTERNATE(3));   /* CH 1 */
	palSetPadMode(GPIOC,  7, PAL_MODE_ALTERNATE(3));   /* CH 2 */
	palSetPadMode(GPIOC,  8, PAL_MODE_ALTERNATE(3));   /* CH 3 */
	palSetPadMode(GPIOC,  9, PAL_MODE_ALTERNATE(3));   /* CH 4 */
}

void update_motors(float *dc)
{
	pwmEnableChannel(&PWMD1, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD1,  dc[0]*10000));
	pwmEnableChannel(&PWMD1, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD1,  dc[1]*10000));
	pwmEnableChannel(&PWMD1, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD1,  dc[2]*10000));
	pwmEnableChannel(&PWMD1, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD1,  dc[3]*10000));
	//pwmEnableChannel(&PWMD2, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD2, dc[0]*10000));
	//pwmEnableChannel(&PWMD2, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD2, dc[1]*10000));
	//pwmEnableChannel(&PWMD2, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD2, dc[2]*10000));
	//pwmEnableChannel(&PWMD2, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD2, dc[3]*10000));
	pwmEnableChannel(&PWMD8, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc[4]*10000));
	pwmEnableChannel(&PWMD8, 1, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc[5]*10000));
	pwmEnableChannel(&PWMD8, 2, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc[6]*10000));
	pwmEnableChannel(&PWMD8, 3, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, dc[7]*10000));
}

