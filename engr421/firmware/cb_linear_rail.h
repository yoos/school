#ifndef CB_LINEAR_RAIL_H
#define CB_LINEAR_RAIL_H

#include <ch.h>
#include <hal.h>
#include <cb_icu.h>
#include <cb_pid.h>
#include <cb_config.h>
#include <cb_math.h>
#include <cb_linear_rail_controller.h>
#include <chsprintf.h>

/**
 * @brief Set up linear rail.
 */
void setup_linear_rail(void);

/**
 * @brief Update linear rail velocity and stuff.
 *
 * @param base_wheel_dc Duty cycle of death ray. Used here just to determine
 * enable status. TODO: Do this properly.
 * @param des_lin_pos Desired linear position.
 *
 * @output dir Direction of movement (0 or 1).
 * @output dc Duty cycle for linear rail motor.
 */
void update_linear_rail(uint8_t enabled, float *des_lin_pos, uint8_t *dir, float *dc);

/**
 * @brief Debug output
 *
 * @param buffer Output buffer.
 */
void linear_rail_debug_output(uint8_t *buffer);

/**
 * @brief Update position of linear rail (i.e., keep track of multiple
 * revolutions of encoder.
 *
 * A given rotational positon of the encoder will fit into certain "slots" in
 * the linear position of the rail. We assume that this position will not
 * change by more than a half rotation between any two timesteps.
 *
 * @output lin_pos Linear position of rail. Also used as an input.
 */
void _update_linear_rail_position(float *lin_pos);

#endif /* CB_LINEAR_RAIL_H */

