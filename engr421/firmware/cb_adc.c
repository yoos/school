#include <cb_adc.h>

/*
 * ADC samples buffer.
 */
static adcsample_t samples[ADC_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

adcsample_t avg_ch[ADC_NUM_CHANNELS];

void setup_adc(void)
{
	/*
	 * Initialize ADC driver 1 and set the following as inputs: PC0, PC1, PC2,
	 * PC3
	 */
	adcStart(&ADCD1, NULL);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
}

void update_adc(void)
{
	chSysLockFromIsr();
	adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
	chSysUnlockFromIsr();
}

/*
 * ADC end conversion callback.
 * The PWM channels are reprogrammed using the latest ADC samples.
 * The latest samples are transmitted into a single SPI transaction.
 */
void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	(void) buffer; (void) n;
	/* Note, only in the ADC_COMPLETE state because the ADC driver fires an
	 * intermediate callback when the buffer is half full. */
	if (adcp->state == ADC_COMPLETE) {
		int i=0;
		for (i=0; i<4; i++) {
			avg_ch[i] = (samples[i] + samples[i+1*ADC_NUM_CHANNELS] + samples[i+2*ADC_NUM_CHANNELS] + samples[i+3*ADC_NUM_CHANNELS]) / 4;
		}
	}
}

