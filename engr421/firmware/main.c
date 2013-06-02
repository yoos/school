#include <ch.h>
#include <hal.h>

#include <chsprintf.h>

#include <cb_adc.h>   // ADC code
#include <cb_comm.h>   // Communications code
#include <cb_icu.h>   // ICU code
#include <cb_motor.h>   // Motor control
#include <cb_death_ray.h>   // Death ray control code
#include <cb_hopper.h>   // Hopper control code
#include <cb_linear_rail.h>   // Linear rail control code
#include <cb_config.h>   // General configuration
#include <cb_math.h>


float dutyCycle = 0;
float base_wheel_dc = 0;
float dc[8];
float lr_des_pos[2];   /* Desired linear rail position from serial input. */
uint8_t digital_state[8];
uint8_t enabled = 0;

void clear_buffer(uint8_t *buffer)
{
	uint16_t i;
	for (i=0; i<sizeof(buffer); i++) {
		buffer[i] = 0;
	}
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
	uint8_t rxbuf[200];

	while (TRUE) {
		time += MS2ST(10);
		counter++;

		uartStopSend(&UARTD3);
		uartStopReceive(&UARTD3);

		///command_t rc = packet_to_command();
		///if (rc.new_command) {
		///	lr_des_pos[0] = rc.one.linear_rail_pos;
		///	lr_des_pos[1] = rc.two.linear_rail_pos;
		///}

		/* Zero out buffer. TODO: Maybe check whether or not we've finished
		 * transmitting? */
		clear_buffer(txbuf);

		//chsprintf(txbuf, "ICU: %6d %6d %6d %6d\r\n", (int) (icu_get_period(2)*1000), (int) (icu_get_period(3)*1000), (int) (icu_get_period(4)*1000), (int) (icu_get_period(5)*1000));
		//chsprintf(txbuf, "%u %6u %6u | %u %u\r\n", (int) rc.new_command, (uint8_t) (ABS(rc.one.linear_rail_pos)*255), (uint8_t) (ABS(rc.two.linear_rail_pos)*255), (uint8_t) (ABS(rc.one.death_ray_intensity*255)), (uint8_t) (ABS(rc.two.death_ray_intensity*255)));
		///comm_debug_output(txbuf);
		///dc[7] = rc.one.linear_rail_pos;
		lr_des_pos[0] = ((float) (0x7f & rxbuf[0])) / 127;   // Use highest-order bit as enable signal.
		enabled = (uint8_t) (rxbuf[0] >> 7);

		//chsprintf(txbuf, "%u %u\r\n", enabled, (int)(lr_des_pos[0]*1000));
		linear_rail_debug_output(txbuf);

		//death_ray_debug_output(base_wheel_dc, txbuf);
		//chsprintf(txbuf, "%s", rxbuf);
		uartStartSend(&UARTD3, sizeof(txbuf), txbuf);

//		clear_buffer(rxbuf);
		uartStartReceive(&UARTD3, sizeof(rxbuf), rxbuf);

		palSetPad(GPIOD, 12);
		chThdSleepMilliseconds(5);
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

		dutyCycle = avg_ch[3] * 500/4096 + 1;   // TODO: The +1 at the end makes this work. Why?
		base_wheel_dc = ((float) avg_ch[3])/4096;

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

	float dr_dc[2];

	while (TRUE) {
		time += MS2ST(1000*DEATH_RAY_DT);

		update_death_ray(enabled, dr_dc);

		dc[I_PWM_DEATH_RAY_0] = dr_dc[0];
		dc[I_PWM_DEATH_RAY_1] = dr_dc[1];

		if (enabled) {
			palSetPad(GPIOD, 15);
		}
		else {
			palClearPad(GPIOD, 15);
		}

		chThdSleepUntil(time);
	}

	return 0;
}

/*
 * Hopper loop
 */
static WORKING_AREA(wa_hopper_thread, 128);
static msg_t hopper_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("hopper");
	systime_t time = chTimeNow();

	float hopper_dc[2];   /* Duty cycle of hopper motor. */

	while (TRUE) {
		time += MS2ST(1000*HOPPER_DT);

		update_hopper(enabled, hopper_dc);

		dc[I_PWM_HOPPER_0] = hopper_dc[0];
		dc[I_PWM_HOPPER_1] = hopper_dc[1];

		chThdSleepUntil(time);
	}

	return 0;
}

/*
 * Linear rail loop
 */
static WORKING_AREA(wa_linear_rail_thread, 128);
static msg_t linear_rail_thread(void *arg)
{
	(void) arg;
	chRegSetThreadName("linear rail");
	systime_t time = chTimeNow();

	uint8_t lr_dir[2];   /* Direction of linear rail motor. */
	float lr_dc[2];   /* Duty cycle of linear rail motor. */

	while (TRUE) {
		time += MS2ST(1000*LINEAR_RAIL_DT);

		update_linear_rail(enabled, lr_des_pos, lr_dir, lr_dc);

		dc[I_PWM_LINEAR_RAIL_0] = lr_dc[0];
		dc[I_PWM_LINEAR_RAIL_1] = lr_dc[1];
		digital_state[I_DIGITAL_LINEAR_RAIL_0] = lr_dir[0];
		digital_state[I_DIGITAL_LINEAR_RAIL_1] = lr_dir[1];

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
	uint8_t i;
	for (i=0; i<8; i++) {
		palSetPadMode(GPIOD, i, PAL_MODE_OUTPUT_PUSHPULL);
	}

	while (TRUE) {
		time += MS2ST(1000*CONTROL_LOOP_DT);   // Next deadline in 1 ms.

		update_motors(dc);

		uint8_t j;
		for (j=0; j<8; j++) {
			if (digital_state[j] == 1) {
				palSetPad(GPIOD, j);
			}
			else {
				palClearPad(GPIOD, j);
			}
		}

		/* Blink status LED. */
		if      (k < MS2ST(50))  palSetPad  (GPIOD, 13);
		else if (k < MS2ST(150)) palClearPad(GPIOD, 13);
		else if (k < MS2ST(200)) palSetPad  (GPIOD, 13);
		else                     palClearPad(GPIOD, 13);

		k = (k+1) % MS2ST(1000);

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

	chThdSleepMilliseconds(10);   /* Wait for initialization. */

	setup_death_ray();

	setup_hopper();

	setup_linear_rail();

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
	chThdCreateStatic(wa_adc_thread, sizeof(wa_adc_thread), NORMALPRIO+1, adc_thread, NULL);

	/*
	 * Create death ray thread.
	 */
	chThdCreateStatic(wa_death_ray_thread, sizeof(wa_death_ray_thread), HIGHPRIO, death_ray_thread, NULL);

	/*
	 * Create hopper thread.
	 */
	chThdCreateStatic(wa_hopper_thread, sizeof(wa_hopper_thread), HIGHPRIO, hopper_thread, NULL);

	/*
	 * Create linear rail thread.
	 */
	chThdCreateStatic(wa_linear_rail_thread, sizeof(wa_linear_rail_thread), HIGHPRIO-1, linear_rail_thread, NULL);

	/*
	 * Create control thread.
	 */
	chThdCreateStatic(wa_control_thread, sizeof(wa_control_thread), HIGHPRIO+1, control_thread, NULL);

	while (TRUE) {
	}

	return 0;
}

