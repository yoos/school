#include <cb_death_ray.h>

static pid_data_t pid_data_wheel_pos[2];
static float cur_wheel_period[2];
static bool up_to_speed[2];   // Is the wheel up to speed?
static uint16_t startup_counter[2];

/**
 * @brief Determine whether or not we are in startup state.
 *
 * In the case that the wheel significantly slower than our target rotational
 * velocity, we wish to output a constant duty cycle DEATH_RAY_STARTUP_DC
 * instead of that output by our PID controller. The use of a counter provides
 * hysteresis that makes the transition between the two states smoother.
 *
 * @param per Period of quadrature signal from wheel encoder.
 * @param i Index of wheel.
 */
static void det_startup(float per, uint8_t i)
{
	/* Update counter and cap. */
	if (per < ROT_PERIOD_ST) {
		startup_counter[i] = MIN(DEATH_RAY_STARTUP_COUNTER_MAX, ++startup_counter[i]);
	}
	else {
		startup_counter[i] = MAX(0, --startup_counter[i]);
	}

	if (startup_counter[i] == DEATH_RAY_STARTUP_COUNTER_MAX) {
		up_to_speed[i] = true;
	}
	else if (startup_counter[i] == 0) {
		up_to_speed[i] = false;
	}
}

void setup_death_ray(void)
{
	uint8_t i;
	for (i=0; i<2; i++) {
		pid_data_wheel_pos[i].Kp = DEATH_RAY_KP;
		pid_data_wheel_pos[i].Ki = DEATH_RAY_KI;
		pid_data_wheel_pos[i].Kd = DEATH_RAY_KD;
		pid_data_wheel_pos[i].dt = DEATH_RAY_DT;
		pid_data_wheel_pos[i].last_val = 0;
	}
}

void update_death_ray(uint8_t enabled, float *dc)
{
	cur_wheel_period[0] = MIN(2000, icu_get_period(I_ICU_DEATH_RAY_0));   /* The 2000 here is arbitrary. */
	cur_wheel_period[1] = MIN(2000, icu_get_period(I_ICU_DEATH_RAY_1));   /* The 2000 here is arbitrary. */

	uint8_t i;
	for (i=0; i<2; i++) {
		/*
		 * The ICU spits out bogus values of 0 and 39 that could be interpreted as
		 * an actual period. Filter these out.
		 */
		if (cur_wheel_period[i] == 0 || cur_wheel_period[i] == 39) {
			cur_wheel_period[i] = 2000;   /* Again, the 2000 is arbitrary. */
		}

		/* Determine startup state. */
		det_startup(cur_wheel_period[i], i);

		/* Controller */
		if (!enabled) {
			/* Disabled */
			dc[i] = ESC_MIN_DC;
			pid_data_wheel_pos[i].I = 0;   /* Zero integral term. */
		}
		else {
			/* Enabled */
			if (!up_to_speed[i]) {
				/* Startup state */
				dc[i] = DEATH_RAY_STARTUP_DC;
				pid_data_wheel_pos[i].I = 0;   /* Zero integral term. */
			}
			else {
				/* Running state */

				/* Cap I term. */
				pid_data_wheel_pos[i].I = MAX(-DEATH_RAY_I_CAP, pid_data_wheel_pos[i].I);
				pid_data_wheel_pos[i].I = MIN( DEATH_RAY_I_CAP, pid_data_wheel_pos[i].I);

				/* PID control */
				dc[i] = MIN(ESC_MAX_DC, (ESC_MIN_DC + MAX(0, calculate_pid(cur_wheel_period[i], ROT_PERIOD_ST, &pid_data_wheel_pos[i]))));
			}
		}
	}
}

void death_ray_debug_output(uint8_t *buffer)
{
	chsprintf(buffer, "Speed: %5d, %5d / %4d  Step: %6d  T: %8d\r\n",
			(int) cur_wheel_period[0],
			(int) cur_wheel_period[1],
			(int) ROT_PERIOD_ST,
			(int) (ROT_SIZE*1000000),
			(int) chTimeNow());
}

