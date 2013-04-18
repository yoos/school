#include <ch.h>
#include <hal.h>

#include <chsprintf.h>

#include <cb_adc.h>   // ADC code
#include <cb_pid.h>   // PID function definition
#include <cb_comm.h>   // Communications code
#include <cb_icu.h>   // ICU code
#include <cb_motor.h>   // Motor control
#include <cb_config.h>   // General configuration
#include <cb_math.h>

static float base_wheel_dc = 0;
static float last_wheel_pos = 0;
static float cur_wheel_pos = 0;
static float des_wheel_pos = 0;
static float cur_wheel_speed = 0;
static float des_wheel_speed = 0;
static float dc[8];

/*
 * Communications loop
 */
static WORKING_AREA(wa_comm_thread, 1280);
static msg_t comm_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("communications 2");
	systime_t time = chTimeNow();
	int counter = 0;

	uint8_t txbuf[200];

	while (TRUE) {
		time += MS2ST(50);
		counter++;

		/* Zero out buffer. */
		uint8_t i;
		for (i=0; i<sizeof(txbuf); i++) {
			txbuf[i] = 0;
		}

		//chsprintf(txbuf, "ICU: %6d %6d %6d %6d\r\n", (int) (icu_get_period(2)*1000), (int) (icu_get_period(3)*1000), (int) (icu_get_period(4)*1000), (int) (icu_get_period(5)*1000));

		chsprintf(txbuf, "Scur: %4d  Sdes: %4d  Pcur: %5d  Pdes: %4d  DC: %3d  ADC: %3d  Step: %6d  T: %8d\r\n",
				(int) (cur_wheel_speed*1000),
				(int) (des_wheel_speed*1000),
				(int) icu_get_period(5),
				(int) ROT_PERIOD_ST,
				(int) (dc[0]*1000),
				(int) (base_wheel_dc*1000),
				(int) (ROT_SIZE*1000000),
				(int) time);
		uartStartSend(&UARTD3, sizeof(txbuf), txbuf);

		palSetPad(GPIOD, 12);
		chThdSleepMilliseconds(10);
		palClearPad(GPIOD, 12);

		chThdSleepUntil(time);
	}

	return 0;
}

/*
 * ADC loop
 */
static WORKING_AREA(wa_adc_thread, 128);
static msg_t adc_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("adc");
	systime_t time = chTimeNow();

	while (TRUE) {
		time += MS2ST(100);

		update_adc();

		uint16_t dutyCycle = avg_ch[3] * 500/4096 + 1;   // TODO: The +1 at the end makes this work. Why?
		base_wheel_dc = ((float) avg_ch[3])/4096/5;   // Divide by 5 for safety.

		palSetPad(GPIOD, 14);
		chThdSleepMilliseconds(20);
		palClearPad(GPIOD, 14);

		chThdSleepUntil(time);
	}

	return 0;
}

/*
 * Control loop
 */
static WORKING_AREA(wa_control_thread, 2048);
static msg_t control_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("control");
	systime_t time = chTimeNow();

	uint16_t j;
	uint16_t k = 0;

	pid_data_t pid_data_wheel_pos;
	pid_data_wheel_pos.Kp = 0.001;
	pid_data_wheel_pos.Ki = 0;
	pid_data_wheel_pos.Kd = 0;
	pid_data_wheel_pos.last_val = 0;
	pid_data_wheel_pos.dt = CONTROL_LOOP_DT;

	while (TRUE) {
		time += MS2ST(1000*CONTROL_LOOP_DT);   // Next deadline in 1 ms.

		cur_wheel_pos = icu_get_duty_cycle(5);
		//des_wheel_pos -= ROT_SIZE;

		//if (cur_wheel_pos - des_wheel_pos > 1) {
		//	des_wheel_pos += 1;
		//}

		if (base_wheel_dc < 0.053) {
			dc[0] = 0.053;
		}
		else {
			//dc[0] = base_wheel_dc + (cur_wheel_pos - des_wheel_pos) * pid_data_wheel_pos.Kp;
			//dc[0] = base_wheel_dc + calculate_pid(cur_wheel_pos, des_wheel_pos, &pid_data_wheel_pos);
			if (icu_get_period(5) > ROT_PERIOD_ST) {
				
				//dc[0] = base_wheel_dc;   // Bang bang, per RVW's suggestion.
				dc[0] = MIN(1, 0.053 + (MAX(0, icu_get_period(5)-ROT_PERIOD_ST)*pid_data_wheel_pos.Kp));
			}
			else {
				dc[0] = 0.053;
			}
		}

		last_wheel_pos = cur_wheel_pos;

		update_motors(dc);

		/* Blink status LED. */
		if (k < MS2ST(50)) {
			palSetPad(GPIOD, 13);
		}
		else if (k < MS2ST(150)) {
			palClearPad(GPIOD, 13);
		}
		else if (k < MS2ST(200)) {
			palSetPad(GPIOD, 13);
		}
		else {
			palClearPad(GPIOD, 13);
		}

		k = (k++) % MS2ST(1000);

		chThdSleepUntil(time);
	}

	return 0;
}


/*
 * Application entry point.
 */
int main(void)
{
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();

	setup_comm();   // TODO: This hangs thread, I think.

	setup_adc();

	setup_icu();

	setup_motors();

	/*
	 * Short delay to let the various setup functions finish.
	 */
	chThdSleepMilliseconds(1);

	/*
	 * Create the second communications thread.
	 */
	chThdCreateStatic(wa_comm_thread, sizeof(wa_comm_thread), NORMALPRIO, comm_thread, NULL);

	/*
	 * Create the ADC thread.
	 */
	chThdCreateStatic(wa_adc_thread, sizeof(wa_adc_thread), NORMALPRIO, adc_thread, NULL);

	/*
	 * Create control thread.
	 */
	chThdCreateStatic(wa_control_thread, sizeof(wa_control_thread), HIGHPRIO, control_thread, NULL);

	while (TRUE) {
	}

	return 0;
}

