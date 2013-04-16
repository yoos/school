#ifndef CB_ICU_H
#define CB_ICU_H

#include <ch.h>
#include <hal.h>

void setup_icu(void);
void icu2widthcb(ICUDriver *icup);
void icu2periodcb(ICUDriver *icup);
void icu3widthcb(ICUDriver *icup);
void icu3periodcb(ICUDriver *icup);
void icu4widthcb(ICUDriver *icup);
void icu4periodcb(ICUDriver *icup);
void icu5widthcb(ICUDriver *icup);
void icu5periodcb(ICUDriver *icup);

static const ICUConfig icu2cfg = {
	ICU_INPUT_ACTIVE_HIGH,
	200000,   /* 200 kHz ICU clock frequency. */
	icu2widthcb,
	icu2periodcb,
	NULL,
	ICU_CHANNEL_1
};

static const ICUConfig icu3cfg = {
	ICU_INPUT_ACTIVE_HIGH,
	200000,   /* 200 kHz ICU clock frequency. */
	icu3widthcb,
	icu3periodcb,
	NULL,
	ICU_CHANNEL_1
};

static const ICUConfig icu4cfg = {
	ICU_INPUT_ACTIVE_HIGH,
	200000,   /* 200 kHz ICU clock frequency. */
	icu4widthcb,
	icu4periodcb,
	NULL,
	ICU_CHANNEL_1
};

static const ICUConfig icu5cfg = {
	ICU_INPUT_ACTIVE_HIGH,
	200000,   /* 200 kHz ICU clock frequency. */
	icu5widthcb,
	icu5periodcb,
	NULL,
	ICU_CHANNEL_1
};

#endif /* CB_ICU_H */

