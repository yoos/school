#include <cb_icu.h>

icucnt_t icu2_last_width, icu2_last_period;
icucnt_t icu3_last_width, icu3_last_period;
icucnt_t icu4_last_width, icu4_last_period;
icucnt_t icu5_last_width, icu5_last_period;

void setup_icu(void)
{
	icuStart(&ICUD2, &icu2cfg);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_PULLDOWN | PAL_MODE_ALTERNATE(1));   /* This needs to be 1, not 2? WTF. */
	icuEnable(&ICUD2);

	icuStart(&ICUD3, &icu3cfg);
	palSetPadMode(GPIOB, 4, PAL_MODE_INPUT_PULLDOWN | PAL_MODE_ALTERNATE(2));
	icuEnable(&ICUD3);

	icuStart(&ICUD4, &icu4cfg);
	palSetPadMode(GPIOB, 6, PAL_MODE_INPUT_PULLDOWN | PAL_MODE_ALTERNATE(2));
	icuEnable(&ICUD4);

	icuStart(&ICUD5, &icu5cfg);
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_PULLDOWN | PAL_MODE_ALTERNATE(2));
	icuEnable(&ICUD5);
}

float icu_get_duty_cycle(uint8_t icu_num)
{
	float output;

	switch (icu_num) {
	case 2:
		output = ((float) icu2_last_width) / ((float) icu2_last_period);
		break;
	case 3:
		output = ((float) icu3_last_width) / ((float) icu3_last_period);
		break;
	case 4:
		output = ((float) icu4_last_width) / ((float) icu4_last_period);
		break;
	case 5:
		output = ((float) icu5_last_width) / ((float) icu5_last_period);
		break;
	default:
		output = 0;
	}

	return output;
}

float icu_get_period(uint8_t icu_num)
{
	float output;

	switch (icu_num) {
	case 2:
		output = (float) icu2_last_period;
		break;
	case 3:
		output = (float) icu3_last_period;
		break;
	case 4:
		output = (float) icu4_last_period;
		break;
	case 5:
		output = (float) icu5_last_period;
		break;
	default:
		output = 0;
	}

	return output;
}

void icu2widthcb(ICUDriver *icup)
{
	//palSetPad(GPIOD, 15);
	icu2_last_width = icuGetWidth(icup);
}

void icu2periodcb(ICUDriver *icup)
{
	palClearPad(GPIOD, 15);
	icu2_last_period = icuGetPeriod(icup);
}

void icu3widthcb(ICUDriver *icup)
{
	//palSetPad(GPIOD, 15);
	icu3_last_width = icuGetWidth(icup);
}

void icu3periodcb(ICUDriver *icup)
{
	palClearPad(GPIOD, 15);
	icu3_last_period = icuGetPeriod(icup);
}

void icu4widthcb(ICUDriver *icup)
{
	//palSetPad(GPIOD, 15);
	icu4_last_width = icuGetWidth(icup);
}

void icu4periodcb(ICUDriver *icup)
{
	palClearPad(GPIOD, 15);
	icu4_last_period = icuGetPeriod(icup);
}

void icu5widthcb(ICUDriver *icup)
{
	//palSetPad(GPIOD, 15);
	icu5_last_width = icuGetWidth(icup);
}

void icu5periodcb(ICUDriver *icup)
{
	palClearPad(GPIOD, 15);
	icu5_last_period = icuGetPeriod(icup);
}

