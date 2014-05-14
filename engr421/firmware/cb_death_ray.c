#include <cb_death_ray.h>

static pid_data_t pid_data_wheel_pos;
static float cur_wheel_period;
static bool up_to_speed;   // Is the wheel up to speed?
//static uint16_t startup_counter;

/**
 * @brief Determine whether or not we are in startup state.
 *
 * In the case that the wheel significantly slower than our target rotational
 * velocity, we wish to output a constant duty cycle DEATH_RAY_STARTUP_DC
 * instead of that output by our PID controller. The use of a counter provides
 * hysteresis that makes the transition between the two states smoother.
 *
 * @param per Period of quadrature signal from wheel encoder.
 */
static void det_startup(float per)
{
	/* Update counter and cap. */
	//if (per < ROT_PERIOD_ST) {
	//	startup_counter[i] = MIN(DEATH_RAY_STARTUP_COUNTER_MAX, ++startup_counter[i]);
	//}
	//else {
	//	startup_counter[i] = MAX(0, --startup_counter[i]);
	//}

	//if (startup_counter[i] == DEATH_RAY_STARTUP_COUNTER_MAX) {
	//	up_to_speed[i] = true;
	//}
	//else if (startup_counter[i] == 0) {
	//	up_to_speed[i] = false;
	//}
	(void) per;
	up_to_speed = true;
}

void setup_death_ray(void)
{
	pid_data_wheel_pos.Kp = DEATH_RAY_KP;
	pid_data_wheel_pos.Ki = DEATH_RAY_KI;
	pid_data_wheel_pos.Kd = DEATH_RAY_KD;
	pid_data_wheel_pos.dt = DEATH_RAY_DT;
	pid_data_wheel_pos.last_val = 0;
}

void update_death_ray(uint8_t status, float *dc)
{
	cur_wheel_period = MIN(2000, icu_get_period(I_ICU_DEATH_RAY));   /* The 2000 here is arbitrary. */

	/*
	 * The ICU spits out bogus values of 0 and 39 that could be interpreted as
	 * an actual period. Filter these out.
	 */
	if (cur_wheel_period == 0 || cur_wheel_period == 39) {
		cur_wheel_period = 2000;   /* Again, the 2000 is arbitrary. */
	}

	/* Determine startup state. */
	det_startup(cur_wheel_period);

	/* Controller */
	if (status == DISABLED) {
		/* Disabled */
		*dc = ESC_MIN_DC;
		pid_data_wheel_pos.I = 0;   /* Zero integral term. */
	}
	else {
		/* Standing by or playing */
		if (!up_to_speed) {
			/* Startup state */
			*dc = DEATH_RAY_STARTUP_DC;
			pid_data_wheel_pos.I = 0;   /* Zero integral term. */
		}
		else {
			/* Running state */

			/* Cap I term. */
			pid_data_wheel_pos.I = MAX(-DEATH_RAY_I_CAP, pid_data_wheel_pos.I);
			pid_data_wheel_pos.I = MIN( DEATH_RAY_I_CAP, pid_data_wheel_pos.I);

			/* PID control. */
			*dc = MIN(ESC_MAX_DC, (ESC_MIN_DC + MAX(0, calculate_pid(cur_wheel_period, ROT_PERIOD_ST, &pid_data_wheel_pos))));
		}
	}
}

void death_ray_debug_output(uint8_t *buffer)
{
	chsprintf(buffer, "Speed: %5d/%4d  Step: %6d  T: %8d\r\n",
			(int) cur_wheel_period,
			(int) ROT_PERIOD_ST,
			(int) (ROT_SIZE*1000000),
			(int) chTimeNow());
}

