#include <ch.h>
#include <hal.h>

#include <chsprintf.h>

#include <cb_adc.h>
#include <cb_pid.h>
#include <cb_comm.h>
#include <cb_motor.h>
#include <cb_config.h>

/*
 * Blue LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

	(void)arg;
	chRegSetThreadName("blinker1");
	while (TRUE) {
		palClearPad(GPIOC, GPIOC_LED4);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOC, GPIOC_LED4);
		chThdSleepMilliseconds(500);
	}
}

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread2, 128);
static msg_t Thread2(void *arg) {

	(void)arg;
	chRegSetThreadName("blinker2");
	while (TRUE) {
		palClearPad(GPIOC, GPIOC_LED3);
		chThdSleepMilliseconds(250);
		palSetPad(GPIOC, GPIOC_LED3);
		chThdSleepMilliseconds(250);
	}
}

/*
 * Application entry point.
 */
int main(void) {

	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *	 and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *	 RTOS is active.
	 */
	halInit();
	chSysInit();

	/*
	 * Creates the blinker threads.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
	chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);

	while (TRUE) {
	}

	return 0;
}

