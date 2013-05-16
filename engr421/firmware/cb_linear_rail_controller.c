#include <cb_linear_rail_controller.h>

void linear_rail_position_controller (pid_data_t *pid_data, float cur_pos, float des_pos, float des_vel)
{
	/* Cap desired position. */
	des_pos = MIN(1.0, MAX(0.0, des_pos));

	/* Calculate desired velocities. */
	des_vel = calculate_pid(cur_pos, des_pos, pid_data);
}

void linear_rail_velocity_controller (pid_data_t *pid_data, float cur_vel, float des_vel, float dc_shift)
{
	/* Cap linear velocity. */
	des_vel = MIN(LINEAR_RAIL_VEL_CAP, MAX(-LINEAR_RAIL_VEL_CAP, des_vel));

	/* Calculate duty cycle shifts. */
	dc_shift = calculate_pid(cur_vel, des_vel, pid_data);
}

void map_to_bounds (float* input, uint8_t input_size, float bound_lower, float bound_upper, float* output)
{
	float map_lower = bound_lower;
	float map_upper = bound_upper;

	/* Find maximum and minimum input values. */
	int i;
	for (i=0; i<input_size; i++) {
		if (map_upper < input[i]) {
			map_upper = input[i];
		}
		else if (map_lower > input[i]) {
			map_lower = input[i];
		}
	}

	/* Limit, but not fit, the inputs to the maximum and minimum values. */
	float offset = ABS(bound_lower - map_lower);
	float scale = 1.0 / (map_upper - map_lower);
	for (i=0; i<input_size; i++) {
		output[i] = (input[i] + offset) * scale;
	}
}

