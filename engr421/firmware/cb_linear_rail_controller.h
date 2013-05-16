#ifndef CB_LINEAR_RAIL_CONTROLLER_H
#define CB_LINEAR_RAIL_CONTROLLER_H

#include <ch.h>
#include <hal.h>
#include <math.h>
#include <cb_config.h>
#include <cb_pid.h>
#include <cb_math.h>

/**
 * @brief Calculate desired velocity based on desired position input.
 *
 * @param pid_data PID data struct.
 * @param cur_pos Current position.
 * @param des_pos Desired position.
 *
 * @output des_vel Desired velocity.
 */
void linear_rail_position_controller (pid_data_t *pid_data, float cur_pos, float des_pos, float des_vel);

/**
 * @brief Calculate shifts in individual duty cycles of motors baseed on
 *     desired angular velocity input.
 *
 * @param pid_data PID data struct.
 * @param cur_vel Current velocity.
 * @param des_vel Desired velocity.
 *
 * @output dc_shift Shift in duty cycle.
 */
void linear_rail_velocity_controller (pid_data_t *pid_data, float cur_vel, float des_vel, float dc_shift);

/**
 * @brief Map input array to be within desired bounds.
 *
 * @param input Input array of at least one element.
 * @param input_size Size of input array.
 * @param bound_lower Desired lower bound.
 * @param bound_upper Desired upper bound.
 *
 * @output output Mapped output array.
 */
void map_to_bounds(float* input, uint8_t input_size, float bound_lower, float bound_upper, float* output);

#endif // CB_LINEAR_RAIL_CONTROLLER_H

