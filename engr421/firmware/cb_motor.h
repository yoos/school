#ifndef CB_MOTOR_H
#define CB_MOTOR_H

#include <ch.h>
#include <hal.h>
#include <cb_config.h>

/**
 * @brief Set up pins.
 */
void setup_motors(void);

/**
 * @brief Send new desired motor speeds to ESCs.
 * @param dc Array of duty cycles to write to motors.
 */
void update_motors(float* dc);

/**
 * @brief PWM configuration structure for ESCs using channels 1, 3, and 4 (PA8, PA10,
 * PA11) of TIM1.
 *
 * These are actually tied to the USB pins but are still usable. PA9 is not
 * used because it is tied to a capacitor and its PWM waveform looks horrible.
 * Funnily enough, if I use it any, the waveforms of all the other PWM lines
 * are affected slightly, as well.
 */
static const PWMConfig pwm1cfg = {
	200000,   // 200 kHz PWM clock frequency.
	1000,    // PWM period 5 ms.
	NULL,    // No callback.
	{
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_DISABLED,    NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL}
	},
	0   // HW dependent
};

/**
 * @brief PWM configuration structure for ESCs using channels 1 through 4 (PC6,
 * PC7, PC8, PC9) of TIM8.
 *
 * See datasheet page 29 for available timers and their capabilities.
 * See datasheet page 45 for pinouts.
 */
static const PWMConfig pwm8cfg = {
	200000,   // 200 kHz PWM clock frequency.
	1000,      // PWM period 5 ms.
	NULL,      // No callback.
	{
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL}
	},
	0   // HW dependent
};

#endif // CB_MOTOR_H

