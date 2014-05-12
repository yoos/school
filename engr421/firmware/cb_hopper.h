#ifndef CB_HOPPER_H
#define CB_HOPPER_H

#include <ch.h>
#include <hal.h>
#include <cb_pid.h>
#include <cb_config.h>
#include <cb_math.h>
#include <chsprintf.h>

/**
 * @brief Set up hopper.
 */
void setup_hopper(void);

/**
 * @brief Update hopper velocity and stuff.
 *
 * @param status Status byte indicating play state.
 *
 * @output dc Duty cycle for hopper motor.
 */
void update_hopper(uint8_t status, float dc);

/**
 * @brief Debug output
 *
 * @param buffer Output buffer.
 */
void hopper_debug_output(uint8_t *buffer);

#endif /* CB_HOPPER_H */

