#include <ch.h>
#include <hal.h>

#include <chsprintf.h>

#include <cb_adc.h>   // ADC code
#include <cb_comm.h>   // Communications code
#include <cb_icu.h>   // ICU code
#include <cb_motor.h>   // Motor control
#include <cb_death_ray.h>   // Death ray control code
#include <cb_config.h>   // General configuration
#include <cb_math.h>


float base_wheel_dc = 0;
float dc[8];

/*
 * Communications loop
 */
static WORKING_AREA(wa_comm_thread, 1280);
static msg_t comm_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("communications");
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

		death_ray_debug_output(txbuf);
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
 * Death Ray loop
 */
static WORKING_AREA(wa_death_ray_thread, 128);
static msg_t death_ray_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("death ray");
	systime_t time = chTimeNow();

	while (TRUE) {
		time += MS2ST(1000*DEATH_RAY_DT);

		update_death_ray(dc, base_wheel_dc);

		chThdSleepUntil(time);
	}

	return 0;
}

/*
 * Control loop
 */
static WORKING_AREA(wa_control_thread, 128);
static msg_t control_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("control");
	systime_t time = chTimeNow();

	uint16_t k = 0;

	while (TRUE) {
		time += MS2ST(1000*CONTROL_LOOP_DT);   // Next deadline in 1 ms.

		update_motors(dc);

		/* Blink status LED. */
		if      (k < MS2ST(50))  palSetPad  (GPIOD, 13);
		else if (k < MS2ST(150)) palClearPad(GPIOD, 13);
		else if (k < MS2ST(200)) palSetPad  (GPIOD, 13);
		else                     palClearPad(GPIOD, 13);

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
	 * Create death ray thread.
	 */
	chThdCreateStatic(wa_death_ray_thread, sizeof(wa_death_ray_thread), HIGHPRIO, death_ray_thread, NULL);

	/*
	 * Create control thread.
	 */
	chThdCreateStatic(wa_control_thread, sizeof(wa_control_thread), HIGHPRIO, control_thread, NULL);

	while (TRUE) {
	}

	return 0;
}

