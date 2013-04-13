#include <cb_adc.h>

/*
 * ADC samples buffer.
 */
static adcsample_t samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

adcsample_t avg_ch[ADC_NUM_CHANNELS];

void setup_adc(void)
{
	/*
	 * Initialize ADC driver 1 and set the following as inputs: TODO: find pins
	 * to use!
	 */
	adcStart(&ADCD1, NULL);

	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOB, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOD, 3, PAL_MODE_INPUT_ANALOG);
}

void update_adc(void)
{
	chSysLockFromIsr();
	adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_BUF_DEPTH);
	chSysUnlockFromIsr();
}

/*
 * ADC end conversion callback.
 */
void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
	(void) buffer; (void) n;
	/* Check that we are in the ADC_COMPLETE state, because the ADC driver
	 * fires an intermediate callback when the buffer is half full. */
	if (adcp->state == ADC_COMPLETE) {
		int i, j;
		for (i=0; i<ADC_NUM_CHANNELS; i++) {
			avg_ch[i] = 0;
			for (j=0; j<ADC_BUF_DEPTH; j++) {
				avg_ch[i] += buffer[i+j];
			}
			avg_ch[i] /= ADC_BUF_DEPTH;
		}
	}
}

