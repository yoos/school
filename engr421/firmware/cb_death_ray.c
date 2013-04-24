#include <cb_death_ray.h>

static pid_data_t pid_data_wheel_pos;

static bool up_to_speed = false;

static float cur_wheel_period = 0;

static float pidV = 0;

static float dutyC = 0;

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
	dutyC = dc[0];
	cur_wheel_period = MIN(2000, icu_get_period(5));
	if (cur_wheel_period == 0 || cur_wheel_period == 39) {
		cur_wheel_period = 2000;
	}

	if (!up_to_speed && base_wheel_dc >= ESC_MIN_DC) {
		/* Startup state */
		dc[0] = DEATH_RAY_STARTUP_DC;
		pid_data_wheel_pos.I = 0;

		if (cur_wheel_period < ROT_PERIOD_ST) {
			up_to_speed = true;
		}
	}
	else {
		/* Running state */
		if (base_wheel_dc < ESC_MIN_DC) {
			/* Disabled */
			dc[0] = ESC_MIN_DC;
			pid_data_wheel_pos.I = 0;
			up_to_speed = false;
		}
		else {
			/* Enabled */

			/* Cap I term. */
			//if (pid_data_wheel_pos.I > DEATH_RAY_I_CAP)
			//	pid_data_wheel_pos.I = DEATH_RAY_I_CAP;
			//else if (pid_data_wheel_pos.I < -DEATH_RAY_I_CAP)
			//	pid_data_wheel_pos.I = -DEATH_RAY_I_CAP;

			//pid_data_wheel_pos.I = MAX(DEATH_RAY_I_CAP, pid_data_wheel_pos.I);
			pidV = calculate_pid(cur_wheel_period, ROT_PERIOD_ST, &pid_data_wheel_pos);

			dc[0] = MIN(ESC_MAX_DC, (ESC_MIN_DC + MAX(0, pidV)));

			//dc[0] = MIN(1, ESC_MIN_DC + (MAX(0, icu_get_period(5)-ROT_PERIOD_ST)*pid_data_wheel_pos.Kp));
		}
	}
}

void death_ray_debug_output(uint8_t *buffer, float base_wheel_dc)
{
	chsprintf(buffer, "ADC: %4d  Speed: %5d/%4d  dc: %4d  PID: %4d  Step: %6d  T: %8d\r\n",
			(int) (base_wheel_dc*1000),
			(int) cur_wheel_period,
			(int) ROT_PERIOD_ST,
			(int) (dutyC*1000),
			(int) (ABS(pidV)*1000),
			(int) (ROT_SIZE*1000000),
			(int) chTimeNow());
}

