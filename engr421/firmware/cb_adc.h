#ifndef CB_ADC_H
#define CB_ADC_H

#include <ch.h>
#include <hal.h>

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_NUM_CHANNELS 4

/* Depth of the conversion buffer, channels are sampled eight times each.*/
#define ADC_BUF_DEPTH 8

/* Average values from ADC samples. */
extern adcsample_t avg_ch[ADC_NUM_CHANNELS];

extern void setup_adc(void);
extern void update_adc(void);
extern void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 8 samples of 4 channels, SW triggered.
 * Channels:    IN4, IN5, IN10, IN11, IN12, IN13 (14 cycles sample time)
 * ===
 * Usage notes:
 *
 * TODO: Figure out what the CR registers do.
 *
 * The SMPR register specifies sample times.
 *
 * The SQR registers specify the sequence in which the ADC driver will sample
 * the channels. Here, the channels are sampled in the order given above. See
 * the definition of ADCConversionGroup in adc_lld.h and bit definitions
 * starting on line 1400 in stm32f4xx.h.
 */
static const ADCConversionGroup adcgrpcfg = {
	FALSE,   /* Linear buffer (TRUE for circular) */
	ADC_NUM_CHANNELS,   /* Number of analog channels belonging to this group */
	adccb,   /* Callback (NULL if none) */
	NULL,   /* Error callback (NULL if none) */
	/* HW dependent part. */
	ADC_CFGR1_RES_12BIT,   /* CFGRR1 */
	ADC_TR(0, 0),   /* TR */
	ADC_SMPR_SMP_1P5,   /* SMPR */
	ADC_CHSELR_CHSEL10 | ADC_CHSELR_CHSEL11 | ADC_CHSELR_CHSEL12 | ADC_CHSELR_CHSEL13   /* CHSELR   TODO: Figure out actually which ADC channels I want to use. */
};

#endif // CB_ADC_H

