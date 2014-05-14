#include <cb_linear_rail.h>

static pid_data_t pid_data_pos;
static pid_data_t pid_data_vel;
static float rot_pos_zero;   /* Zero position. Rail should be placed at end of slide at setup. TODO: This should be done with limit switches. */
static float cur_lin_pos;   /* Current position of linear rail. */
static float cur_lin_vel;   /* Current velocity of linear rail. */
static float lin_rot_max;

// DEBUG
static uint8_t dbg_status;
static float dbg_rot_pos;
static float des_pos;
static uint8_t des_dir;
static float des_dc;
static uint16_t dbg_dc_shift;
static uint16_t dbg_q, dbg_r;
static uint8_t dbg_mode;

void setup_linear_rail(void)
{
	rot_pos_zero = 1.0 - icu_get_duty_cycle(I_ICU_LINEAR_RAIL);
	cur_lin_pos = 0.0;
	cur_lin_vel = 0.0;
	lin_rot_max = MIN_REVS_PER_LENGTH;

	pid_data_pos.Kp = LINEAR_RAIL_POS_KP;
	pid_data_pos.Ki = LINEAR_RAIL_POS_KI;
	pid_data_pos.Kd = LINEAR_RAIL_POS_KD;
	pid_data_pos.dt = LINEAR_RAIL_DT;
	pid_data_pos.last_val = cur_lin_pos;
	pid_data_vel.Kp = LINEAR_RAIL_VEL_KP;
	pid_data_vel.Ki = LINEAR_RAIL_VEL_KI;
	pid_data_vel.Kd = LINEAR_RAIL_VEL_KD;
	pid_data_vel.dt = LINEAR_RAIL_DT;
	pid_data_vel.last_val = cur_lin_vel;
}

uint8_t calibrate_linear_rail(uint8_t status, uint8_t limit_switch, uint8_t *dir, float *dc)
{
	if (cur_lin_pos < 1.0) {
		update_linear_rail(status, MODE_VEL, 1.0, dir, dc);
	}
	else if (limit_switch == 1) {
		update_linear_rail(status, MODE_VEL, LINEAR_RAIL_CALIB_SPEED, dir, dc);
	}
	else {
		lin_rot_max = cur_lin_pos*lin_rot_max;
		cur_lin_pos = 1.0;
		return 1;
	}

	return 0;
}

void update_linear_rail(uint8_t status, uint8_t mode, float target, uint8_t *dir, float *dc)
{
	dbg_status = status;
	static float des_lin_vel;
	static float dc_shift;
	static float rot_pos, q, r, d_rot;

	/*
	 * Calculate linear position from encoder rotational position.
	 *
	 * A given rotational positon of the encoder will fit into certain "slots" in
	 * the linear position of the rail. We assume that this position will not
	 * change by more than 90% of full rotation between any two timesteps. There is
	 * a 10% buffer in the opposite direction of desired movement to counter any
	 * bumps.
	 */
	rot_pos = 1.0 - icu_get_duty_cycle(I_ICU_LINEAR_RAIL);   /* I previously noted here that the ICU spits out bogus values of 0 and 39 that could be misinterpreted as an actual period. Hopefully it doesn't haunt us later. */
	q = (int) (lin_rot_max*cur_lin_pos);   /* Quotient */
	r = lin_rot_max*cur_lin_pos - q;   /* Remainder. Effectively equals old rotational position. */

	/* Debug */
	dbg_rot_pos = rot_pos;
	dbg_q = q;
	dbg_r = r*1000;   /* This should equal current rotational position. */

	/* Track rotation. For now we assume we do not rotate more than half
	 * a revolution per timestep. TODO: Make this work for 90% full rotation
	 * per timestep. */
	d_rot = rot_pos - rot_pos_zero - r;
	if (d_rot > 0.5) {
		d_rot -= 1.0;
	}
	else if (d_rot < -0.5) {
		d_rot += 1.0;
	}

	/* Update current state. */
	cur_lin_pos += d_rot/lin_rot_max;
	cur_lin_vel = d_rot;

	/* Sanity check position target. */
	if (mode == MODE_POS) {
		if (target > 1.0) {
			target = 1.0;
		}
		else if (target < 0.0) {
			target = 0.0;
		}
	}

	/* Run controller */
	if (status == DISABLED) {
		/* Disabled */
		*dir = 0;
		*dc = LINEAR_RAIL_DC_MIN;

		/* Zero integral terms. */
		pid_data_pos.I = 0;
		pid_data_vel.I = 0;
	}
	else {
		/* Standing by or playing */

		/* Cap I term. */
		pid_data_pos.I = MIN(LINEAR_RAIL_POS_I_CAP, MAX(-LINEAR_RAIL_POS_I_CAP, pid_data_pos.I));
		pid_data_vel.I = MIN(LINEAR_RAIL_VEL_I_CAP, MAX(-LINEAR_RAIL_VEL_I_CAP, pid_data_vel.I));

		/* Control */
		if (mode == MODE_POS) {
			des_lin_vel = linear_rail_position_controller(&pid_data_pos, cur_lin_pos, target);
			*dir = (target > cur_lin_pos) ? 1 : 0;
		}
		else {
			des_lin_vel = target;
			*dir = (target > cur_lin_vel) ? 1 : 0;
		}
		dc_shift = linear_rail_velocity_controller(&pid_data_vel, cur_lin_vel, des_lin_vel);

		/* Set duty cycle */
		if (*dir == 0) {
			*dc = MIN(1.0, ABS(dc_shift));
		}
		else {
			*dc = MAX(0.0, 1.0-ABS(dc_shift));   /* Invert duty cycle if reverse. */
		}
	}
	des_pos = target*1000;
	des_dir = *dir;
	des_dc = *dc;
	dbg_dc_shift = (uint16_t) ABS(dc_shift * 1000);
	dbg_mode = mode;
}

void linear_rail_debug_output(uint8_t *buffer)
{
	chsprintf(buffer, "Status: %u  mode: %u  lin pos: %u  rot: %u  des dc: %u  dc_shift: %u  q:  %u  r: %u  des lin: %u\r\n", dbg_status, dbg_mode, (uint16_t) (cur_lin_pos*1000), (uint16_t) (dbg_rot_pos*1000), (uint16_t) (des_dc*1000), dbg_dc_shift, dbg_q, dbg_r, (uint16_t) des_pos);
}

