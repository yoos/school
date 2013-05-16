#include <cb_linear_rail.h>

static pid_data_t pid_data_pos[2];   /* Two rails. */
static pid_data_t pid_data_vel[2];   /* Two rails. */
static float rot_pos_zero[2];   /* Zero position. Rail should be placed at end of slide at setup. TODO: This should be done with limit switches. */
static float cur_lin_pos[2];   /* Current position of linear rail. */
static float cur_lin_vel[2];   /* Current velocity of linear rail. */

void setup_linear_rail(void)
{
	uint8_t i;

	rot_pos_zero[0] = icu_get_duty_cycle(I_ICU_LINEAR_RAIL_0);
	rot_pos_zero[1] = icu_get_duty_cycle(I_ICU_LINEAR_RAIL_1);

	for (i=0; i<2; i++) {
		pid_data_pos[i].Kp = LINEAR_RAIL_POS_KP;
		pid_data_pos[i].Ki = LINEAR_RAIL_POS_KI;
		pid_data_pos[i].Kd = LINEAR_RAIL_POS_KD;
		pid_data_pos[i].dt = LINEAR_RAIL_DT;
		pid_data_pos[i].last_val = 0;
		pid_data_vel[i].Kp = LINEAR_RAIL_VEL_KP;
		pid_data_vel[i].Ki = LINEAR_RAIL_VEL_KI;
		pid_data_vel[i].Kd = LINEAR_RAIL_VEL_KD;
		pid_data_vel[i].dt = LINEAR_RAIL_DT;
		pid_data_vel[i].last_val = 0;

		cur_lin_pos[i] = 0;
		cur_lin_vel[i] = 0;
	}
}

void update_linear_rail(float base_wheel_dc, float *des_lin_pos, uint8_t *dir, float *dc)
{
	uint8_t i;

	static float des_lin_vel[2];
	static float dc_shift[2];

	_update_linear_rail_position(cur_lin_pos);

	/* Controller */
	if (base_wheel_dc < ESC_MIN_DC) {
		/* Disabled */
		for (i=0; i<2; i++) {
			dc[i] = LINEAR_RAIL_DC_MIN;

			/* Zero integral terms. */
			pid_data_pos[i].I = 0;
			pid_data_vel[i].I = 0;
		}
	}
	else {
		/* Enabled */
		for (i=0; i<2; i++) {
			/* Cap I term. */
			pid_data_pos[i].I = MIN(LINEAR_RAIL_POS_I_CAP, MAX(-LINEAR_RAIL_POS_I_CAP, pid_data_pos[i].I));
			pid_data_vel[i].I = MIN(LINEAR_RAIL_VEL_I_CAP, MAX(-LINEAR_RAIL_VEL_I_CAP, pid_data_vel[i].I));

			/* Control */
			linear_rail_position_controller(&pid_data_pos[i], cur_lin_pos[i], des_lin_pos[i], des_lin_vel[i]);
			linear_rail_velocity_controller(&pid_data_vel[i], cur_lin_vel[i], des_lin_vel[i], dc_shift[i]);

			dir[i] = (dc_shift[i] < 0) ? 0 : 1;
			if (dir[i] == 0) {
				dc[i] = MIN(1.0, ABS(dc_shift[i]));
			}
			else {
				dc[i] = MAX(0.0, 1.0-ABS(dc_shift[i]));   /* Invert duty cycle if reverse. */
			}
		}
	}
}

void linear_rail_debug_output(uint8_t *buffer)
{
	chsprintf(buffer, "LR\r\n");
}

void _update_linear_rail_position(float *lin_pos)
{
	uint8_t i;

	float rot_pos[2];
	rot_pos[0] = icu_get_duty_cycle(I_ICU_LINEAR_RAIL_0);
	rot_pos[1] = icu_get_duty_cycle(I_ICU_LINEAR_RAIL_1);

	/*
	 * The ICU spits out bogus values of 0 and 39 that could be interpreted as
	 * an actual period. Filter these out.
	 *
	 * This may or may not be needed for the duty cycle getter. The AS5040
	 * absolute position output sometimes spikes. I may need to sanity check.
	 */
	// for (i=0; i<2; i++) {
	// 	if (cur_pwm_period[i] == 0 || cur_pwm_period[i] == 39) {
	// 		cur_pwm_period[i] = 2000;   /* Again, the 2000 is arbitrary. */
	// 	}
	// }

	float q[2];   /* Quotient */
	float r[2];   /* Remainder. Effectively equals old rotational position. */
	for (i=0; i<2; i++) {
		q[i] = (int) (REVS_PER_LENGTH*lin_pos[i] - rot_pos_zero[i]);
		r[i] = (REVS_PER_LENGTH*lin_pos[i] - rot_pos_zero[i] - q[i]);

		/* Calculate rotation. */
		float d_rot = rot_pos[i] - r[i];
		if (d_rot > 0.5) {
			d_rot -= 1.0;
		}
		else if (d_rot < -0.5) {
			d_rot += 1.0;
		}

		/* Calculate new position. */
		lin_pos[i] += d_rot/REVS_PER_LENGTH;
	}
}

