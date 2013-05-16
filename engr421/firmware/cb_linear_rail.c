#include <cb_linear_rail.h>

static pid_data_t pid_data_pos[2];   /* Two rails. */
static pid_data_t pid_data_vel[2];   /* Two rails. */
static float cur_lin_pos[2];   /* Current position of linear rail. */
static float cur_lin_vel[2];   /* Current velocity of linear rail. */

void setup_linear_rail(void)
{
	uint8_t i;
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
			dc[i] = MIN(1.0, ABS(dc_shift[i]));
		}
	}
}

void linear_rail_debug_output(uint8_t *buffer)
{
	chsprintf(buffer, "LR\r\n");
}

void _update_linear_rail_position(float *lin_pos)
{
	float rot_pos[2];
	rot_pos[0] = icu_get_duty_cycle(I_ICU_LINEAR_RAIL_0);
	rot_pos[1] = icu_get_duty_cycle(I_ICU_LINEAR_RAIL_1);

	/*
	 * The ICU spits out bogus values of 0 and 39 that could be interpreted as
	 * an actual period. Filter these out.
	 *
	 * This may or may not be needed for the duty cycle getter.
	 */
	// for (i=0; i<2; i++) {
	// 	if (cur_pwm_period[i] == 0 || cur_pwm_period[i] == 39) {
	// 		cur_pwm_period[i] = 2000;   /* Again, the 2000 is arbitrary. */
	// 	}
	// }
}

