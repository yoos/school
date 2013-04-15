#include <ch.h>
#include <hal.h>

#include <chsprintf.h>

#include <cb_adc.h>   // ADC code
#include <cb_pid.h>   // PID function definition
#include <cb_comm.h>   // Communications code
#include <cb_motor.h>   // Motor control
#include <cb_config.h>   // General configuration


/*
 * Blink orange LED.
 */
static WORKING_AREA(wa_led_thread, 128);
static msg_t led_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("blinker");
	systime_t time = chTimeNow();

	while (TRUE) {
		time += MS2ST(1000);   // Next deadline in 1 second.

		palSetPad(GPIOD, GPIOD_LED3);
		chThdSleepMilliseconds(50);
		palClearPad(GPIOD, GPIOD_LED3);
		chThdSleepMilliseconds(100);
		palSetPad(GPIOD, GPIOD_LED3);
		chThdSleepMilliseconds(50);
		palClearPad(GPIOD, GPIOD_LED3);

		chThdSleepUntil(time);
	}

	return 0;
}

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
		time += MS2ST(100);
		counter++;

		/* Zero out buffer. */
		uint8_t i;
		for (i=0; i<sizeof(txbuf); i++) {
			txbuf[i] = 0;
		}

		chsprintf(txbuf, "Hello world!");
		uartStartSend(&UARTD1, sizeof(txbuf), txbuf);

		palSetPad(GPIOD, GPIOD_LED4);
		chThdSleepMilliseconds(50);
		palClearPad(GPIOD, GPIOD_LED4);

		chThdSleepUntil(time);
	}

	return 0;
}

/*
 * Second communications loop
 */
static WORKING_AREA(wa_comm_thread_2, 1280);
static msg_t comm_thread_2(void *arg)
{
	(void) arg;
	chRegSetThreadName("communications 2");
	systime_t time = chTimeNow();
	int counter = 0;

	uint8_t txbuf[50];

	/* Zero out buffer. */
	uint8_t i;
	for (i=0; i<sizeof(txbuf); i++) {
		txbuf[i] = 0;
	}

	while (TRUE) {
		time += MS2ST(234);
		counter++;

		chsprintf(txbuf, "Je vis! %d %d %d %d\r\n", 1, 2, 3, 4);
		uartStartSend(&UARTD3, sizeof(txbuf), txbuf);

		palSetPad(GPIOD, GPIOD_LED5);
		chThdSleepMilliseconds(50);
		palClearPad(GPIOD, GPIOD_LED5);

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
		time += MS2ST(500);

		update_adc();

		uint16_t dutyCycle = avg_ch[3] * 500/4096 + 1;   // TODO: The +1 at the end makes this work. Why?

		palSetPad(GPIOD, 15);
		chThdSleepMilliseconds(dutyCycle);
		palClearPad(GPIOD, 15);

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

	setup_motors();

	systime_t time = chTimeNow();
	float i = 0;
	uint8_t j;
	float dir = 0.2;
	float dc[8];

	while (TRUE) {
		time += MS2ST(1);   // Next deadline in 1 ms.   TODO: Any sooner than this, and I2C stops working.
		i += dir;
		if (i > 1000.0) dir = -0.2;
		if (i < 0.0) dir = 0.2;

		for (j=0; j<8; j++) {
			dc[j] = i;
		}

		update_motors(dc);

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

	/*
	 * Short delay to let the various setup functions finish.
	 */
	chThdSleepMilliseconds(1);

	/*
	 * Create the LED thread.
	 */
	chThdCreateStatic(wa_led_thread, sizeof(wa_led_thread), NORMALPRIO, led_thread, NULL);

	/*
	 * Create the communications thread.
	 */
	chThdCreateStatic(wa_comm_thread, sizeof(wa_comm_thread), NORMALPRIO, comm_thread, NULL);

	/*
	 * Create the second communications thread.
	 */
	chThdCreateStatic(wa_comm_thread_2, sizeof(wa_comm_thread_2), NORMALPRIO, comm_thread_2, NULL);

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

