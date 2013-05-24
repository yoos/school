#include <cb_linear_rail.h>

static pid_data_t pid_data_pos[2];   /* Two rails. */
static pid_data_t pid_data_vel[2];   /* Two rails. */
static float rot_pos_zero[2];   /* Zero position. Rail should be placed at end of slide at setup. TODO: This should be done with limit switches. */
static float cur_lin_pos[2];   /* Current position of linear rail. */
static float cur_lin_vel[2];   /* Current velocity of linear rail. */

// DEBUG
static float cur_rot_pos;
static float des_pos;
static uint8_t des_dir;
static float des_dc;
static uint16_t dbg_dc_shift;
static uint16_t pos_ctrl_output, vel_ctrl_output;
static uint16_t dbg_q, dbg_r;

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

void update_linear_rail(uint8_t enabled, float *des_lin_pos, uint8_t *dir, float *dc)
{
	uint8_t i;

	static float des_lin_vel[2];
	static float dc_shift[2];

	_update_linear_rail_position(cur_lin_pos);

	/* Controller */
	if (enabled) {
		/* Disabled */
		for (i=0; i<2; i++) {
			dir[i] = 0;
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
			des_lin_vel[i] = linear_rail_position_controller(&pid_data_pos[i], cur_lin_pos[i], des_lin_pos[i]);
			dc_shift[i]    = linear_rail_velocity_controller(&pid_data_vel[i], cur_lin_vel[i], des_lin_vel[i]);

			dir[i] = (cur_lin_pos[i] > des_lin_pos[i]) ? 0 : 1;
			if (dir[i] == 0) {
				dc[i] = MIN(1.0, ABS(dc_shift[i]));
			}
			else {
				dc[i] = MAX(0.0, 1.0-ABS(dc_shift[i]));   /* Invert duty cycle if reverse. */
			}
		}
	}
	des_pos = des_lin_pos[0];
	des_dir = dir[0];
	des_dc = dc[0];
	dbg_dc_shift = (uint16_t) ABS(dc_shift[0] * 1000);
	pos_ctrl_output = (uint16_t) (des_lin_vel[0] * 1000);

}

void linear_rail_debug_output(uint8_t *buffer)
{
	chsprintf(buffer, "rot pos zero: %u  cur lin pos: %u  des lin pos: %u  cur rot pos: %u  des dir: %u  des dc: %u  dc_shift: %u  pos_ctrl_output: %u  q:  %u  r: %u\r\n", (uint16_t) (rot_pos_zero[0]*1000), (uint16_t) (cur_lin_pos[0]*1000), (uint16_t) (des_pos*1000), (uint16_t) (cur_rot_pos*1000), (uint8_t) des_dir, (uint16_t) (des_dc*1000), dbg_dc_shift, pos_ctrl_output, dbg_q, dbg_r);
}

void _update_linear_rail_position(float *lin_pos)
{
	uint8_t i;

	float rot_pos[2];
	rot_pos[0] = icu_get_duty_cycle(I_ICU_LINEAR_RAIL_0);
	rot_pos[1] = icu_get_duty_cycle(I_ICU_LINEAR_RAIL_1);

	cur_rot_pos = rot_pos[0];

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
		q[i] = (int) (REVS_PER_LENGTH*lin_pos[i] + rot_pos_zero[i]);
		r[i] = (REVS_PER_LENGTH*lin_pos[i] + rot_pos_zero[i] - q[i]);

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

	dbg_q = q[0];
	dbg_r = r[0]*1000;   /* This should equal current rotational position. */
}

