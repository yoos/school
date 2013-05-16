#include <cb_linear_rail.h>

static pid_data_t pid_data_linear_rail[2];   /* Two rails. */
static float cur_pwm_period[2];

void setup_linear_rail(void)
{
	uint8_t i;
	for (i=0; i<2; i++) {
		pid_data_linear_rail[i].Kp = LINEAR_RAIL_KP;
		pid_data_linear_rail[i].Ki = LINEAR_RAIL_KI;
		pid_data_linear_rail[i].Kd = LINEAR_RAIL_KD;
		pid_data_linear_rail[i].dt = LINEAR_RAIL_DT;
		pid_data_linear_rail[i].last_val = 0;

		cur_pwm_period[i] = 0;
	}
}

void update_linear_rail(float *dc, float base_wheel_dc)
{
	uint8_t i;

	cur_pwm_period[0] = MIN(2000, icu_get_period(I_ICU_LINEAR_RAIL_0));   /* The 2000 here is arbitrary. */
	cur_pwm_period[1] = MIN(2000, icu_get_period(I_ICU_LINEAR_RAIL_1));   /* The 2000 here is arbitrary. */

	/*
	 * The ICU spits out bogus values of 0 and 39 that could be interpreted as
	 * an actual period. Filter these out.
	 */
	for (i=0; i<2; i++) {
		if (cur_pwm_period[i] == 0 || cur_pwm_period[i] == 39) {
			cur_pwm_period[i] = 2000;   /* Again, the 2000 is arbitrary. */
		}
	}

	/* Controller */
	if (base_wheel_dc < ESC_MIN_DC) {
		/* Disabled */
		for (i=0; i<2; i++) {
			dc[i] = LINEAR_RAIL_MIN_DC;
			pid_data_linear_rail[i].I = 0;   /* Zero integral term. */
		}
	}
	else {
		/* Enabled */
		for (i=0; i<2; i++) {
			/* Cap I term. */
			pid_data_linear_rail[i].I = MAX(-LINEAR_RAIL_I_CAP, pid_data_linear_rail[i].I);
			pid_data_linear_rail[i].I = MIN( LINEAR_RAIL_I_CAP, pid_data_linear_rail[i].I);

			linear_rail_position_controller(cur_pos[i], des_pos[i], des_vel[i]);
			linear_rail_velocity_controller(cur_vel[i], des_vel[i], dc_shift[i]);

			/* PID control */
			dc[i] = MIN(LINEAR_RAIL_MAX_DC, (LINEAR_RAIL_MIN_DC + MAX(0, calculate_pid(cur_pwm_period[i], , &pid_data_linear_rail[i]))));

			//dc[0] = MIN(1, ESC_MIN_DC + (MAX(0, icu_get_period(5)-ROT_PERIOD_ST)*pid_data_linear_rail.Kp));
		}
	}
}

void linear_rail_debug_output(uint8_t *buffer, float base_wheel_dc)
{
	chsprintf(buffer, "ADC: %4d  Speed: %5d/%4d  Step: %6d  T: %8d\r\n",
			(int) (base_wheel_dc*1000),
			(int) cur_pwm_period,
			(int) ROT_PERIOD_ST,
			(int) (ROT_SIZE*1000000),
			(int) chTimeNow());
}

