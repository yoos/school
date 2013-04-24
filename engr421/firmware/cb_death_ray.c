#include <cb_death_ray.h>

static pid_data_t pid_data_wheel_pos;

static float cur_wheel_period = 0;

static bool up_to_speed = false;   // Is the wheel up to speed?
static uint16_t startup_counter = 0;

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
	if (per < ROT_PERIOD_ST) {
		startup_counter = MIN(DEATH_RAY_STARTUP_COUNTER_MAX, ++startup_counter);
	}
	else {
		startup_counter = MAX(0, --startup_counter);
	}

	if (startup_counter == DEATH_RAY_STARTUP_COUNTER_MAX) {
		up_to_speed = true;
	}
	else if (startup_counter == 0) {
		up_to_speed = false;
	}
}

void setup_death_ray(void)
{
	pid_data_wheel_pos.Kp = DEATH_RAY_KP;
	pid_data_wheel_pos.Ki = DEATH_RAY_KI;
	pid_data_wheel_pos.Kd = DEATH_RAY_KD;
	pid_data_wheel_pos.dt = DEATH_RAY_DT;
	pid_data_wheel_pos.last_val = 0;
}

void update_death_ray(float *dc, float base_wheel_dc)
{
	cur_wheel_period = MIN(2000, icu_get_period(5));   /* The 2000 here is arbitrary. */

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
	if (base_wheel_dc < ESC_MIN_DC) {
		/* Disabled */
		dc[0] = ESC_MIN_DC;
		pid_data_wheel_pos.I = 0;   /* Zero integral term. */
	}
	else {
		/* Enabled */
		if (!up_to_speed) {
			/* Startup state */
			dc[0] = DEATH_RAY_STARTUP_DC;
			pid_data_wheel_pos.I = 0;   /* Zero integral term. */
		}
		else {
			/* Running state */

			/* Cap I term. */
			pid_data_wheel_pos.I = MAX(-DEATH_RAY_I_CAP, pid_data_wheel_pos.I);
			pid_data_wheel_pos.I = MIN( DEATH_RAY_I_CAP, pid_data_wheel_pos.I);

			/* PID control */
			dc[0] = MIN(ESC_MAX_DC, (ESC_MIN_DC + MAX(0, calculate_pid(cur_wheel_period, ROT_PERIOD_ST, &pid_data_wheel_pos))));

			//dc[0] = MIN(1, ESC_MIN_DC + (MAX(0, icu_get_period(5)-ROT_PERIOD_ST)*pid_data_wheel_pos.Kp));
		}
	}
}

void death_ray_debug_output(uint8_t *buffer, float base_wheel_dc)
{
	chsprintf(buffer, "ADC: %4d  Speed: %5d/%4d  Step: %6d  T: %8d\r\n",
			(int) (base_wheel_dc*1000),
			(int) cur_wheel_period,
			(int) ROT_PERIOD_ST,
			(int) (ROT_SIZE*1000000),
			(int) chTimeNow());
}

