#include <cb_death_ray.h>

float last_wheel_pos = 0;
float cur_wheel_pos = 0;
float des_wheel_pos = 0;
float cur_wheel_speed = 0;
float des_wheel_speed = 0;

static pid_data_t pid_data_wheel_pos = {
	0,       // ID
	DEATH_RAY_KP,   // Kp
	DEATH_RAY_KI,   // Ki
	DEATH_RAY_KD,   // Kd
	DEATH_RAY_DT,   // dt
	0.0,     // last_val
	0.0,     // P term
	0.0,     // I term
	0.0      // D term
};


void update_death_ray(float *dc, float base_wheel_dc)
{
	cur_wheel_pos = icu_get_duty_cycle(5);
	cur_wheel_speed = icu_get_period(5);

	if (base_wheel_dc < ESC_MIN_DC) {
		dc[0] = ESC_MIN_DC;
	}
	else {
		/* Cap I term. */
		pid_data_wheel_pos.I = MAX(DEATH_RAY_I_CAP, pid_data_wheel_pos.I);
		//if (icu_get_period(5) > ROT_PERIOD_ST*2) {
		//	pid_data_wheel_pos.I = 0;
		//}

		dc[0] = MIN(1.0, (ESC_MIN_DC + MAX(0, calculate_pid(cur_wheel_speed, ROT_PERIOD_ST, &pid_data_wheel_pos))));

		//dc[0] = MIN(1, ESC_MIN_DC + (MAX(0, icu_get_period(5)-ROT_PERIOD_ST)*pid_data_wheel_pos.Kp));
	}

	last_wheel_pos = cur_wheel_pos;
}

void death_ray_debug_output(uint8_t *buffer)
{
	chsprintf(buffer, "Scur: %5d  Sdes: %4d  Step: %6d  T: %8d\r\n",
			(int) cur_wheel_speed,
			(int) ROT_PERIOD_ST,
			(int) (ROT_SIZE*1000000),
			(int) chTimeNow());
}

