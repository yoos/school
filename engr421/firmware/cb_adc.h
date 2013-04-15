#ifndef OSUAR_ADC_H
#define OSUAR_ADC_H

#include <ch.h>
#include <hal.h>

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_NUM_CHANNELS   6

/* Depth of the conversion buffer, channels are sampled four times each.*/
#define ADC_GRP1_BUF_DEPTH      4

/* Average values from ADC samples. */
extern adcsample_t avg_ch[ADC_NUM_CHANNELS];

extern void setup_adc(void);
extern void update_adc(void);
extern void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 6 channels, SW triggered.
 * Channels:    IN4, IN5, IN10, IN11, IN12, IN13 (48 cycles sample time)
 * ===
 * Usage notes:
 *
 * TODO: Figure out what the CR registers do.
 *
 * The SMPR registers specify sample times. SMPR1 is for channels 10--18. SMPR2
 * is for channels 0--9. Since we use channels in both ranges, we specify both
 * SMPR registers.
 *
 * The SQR registers specify the sequence in which the ADC driver will sample
 * the channels. Here, the channels are sampled in the order given above. See
 * the definition of ADCConversionGroup in adc_lld.h and bit definitions
 * starting on line 1400 in stm32f4xx.h.
 */
static const ADCConversionGroup adcgrpcfg = {
	FALSE,   // Linear buffer (TRUE for circular)
	ADC_NUM_CHANNELS,   // Number of analog channels belonging to this group
	adccb,   // Callback (NULL if none)
	NULL,   // Error callback (NULL if none)
	/* HW dependent part.*/
	0,   // CR1 initialization data
	ADC_CR2_SWSTART,   // CR2 initialization data
	ADC_SMPR1_SMP_AN13(ADC_SAMPLE_56) |
	ADC_SMPR1_SMP_AN12(ADC_SAMPLE_56) |
	ADC_SMPR1_SMP_AN11(ADC_SAMPLE_56) |
	ADC_SMPR1_SMP_AN10(ADC_SAMPLE_56) ,   // SMPR1 initialization data
	ADC_SMPR2_SMP_AN5(ADC_SAMPLE_56)  |
	ADC_SMPR2_SMP_AN4(ADC_SAMPLE_56)  ,   // SMPR2 initialization data
	ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS) ,   // SQR1 initialization data
	ADC_SQR2_SQ7_N(ADC_CHANNEL_IN13)  ,   // SQR2 initialization data
	ADC_SQR3_SQ5_N(ADC_CHANNEL_IN12)  |
	ADC_SQR3_SQ4_N(ADC_CHANNEL_IN11)  |
	ADC_SQR3_SQ3_N(ADC_CHANNEL_IN10)  |
	ADC_SQR3_SQ2_N(ADC_CHANNEL_IN5)   |
	ADC_SQR3_SQ1_N(ADC_CHANNEL_IN4)       // SQR3 initialization data
};

#endif // OSUAR_ADC_H

