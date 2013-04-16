#ifndef CB_ICU_H
#define CB_ICU_H

#include <ch.h>
#include <hal.h>

/**
 * @brief Set up ICU.
 */
void setup_icu(void);

/**
 * @brief Callback at beginning of ICU2 active period.
 */
void icu2widthcb(ICUDriver *icup);

/**
 * @brief Callback at beginning of ICU2 inactive period.
 */
void icu2periodcb(ICUDriver *icup);

/**
 * @brief Callback at beginning of ICU3 active period.
 */
void icu3widthcb(ICUDriver *icup);

/**
 * @brief Callback at beginning of ICU3 inactive period.
 */
void icu3periodcb(ICUDriver *icup);

/**
 * @brief Callback at beginning of ICU4 active period.
 */
void icu4widthcb(ICUDriver *icup);

/**
 * @brief Callback at beginning of ICU4 inactive period.
 */
void icu4periodcb(ICUDriver *icup);

/**
 * @brief Callback at beginning of ICU5 active period.
 */
void icu5widthcb(ICUDriver *icup);

/**
 * @brief Callback at beginning of ICU5 inactive period.
 */
void icu5periodcb(ICUDriver *icup);

/**
 * @brief Configuration struct for ICU on TIM2.
 */
static const ICUConfig icu2cfg = {
	ICU_INPUT_ACTIVE_HIGH,
	200000,   /* 200 kHz ICU clock frequency. */
	icu2widthcb,
	icu2periodcb,
	NULL,
	ICU_CHANNEL_1
};

/**
 * @brief Configuration struct for ICU on TIM3.
 */
static const ICUConfig icu3cfg = {
	ICU_INPUT_ACTIVE_HIGH,
	200000,   /* 200 kHz ICU clock frequency. */
	icu3widthcb,
	icu3periodcb,
	NULL,
	ICU_CHANNEL_1
};

/**
 * @brief Configuration struct for ICU on TIM4.
 */
static const ICUConfig icu4cfg = {
	ICU_INPUT_ACTIVE_HIGH,
	200000,   /* 200 kHz ICU clock frequency. */
	icu4widthcb,
	icu4periodcb,
	NULL,
	ICU_CHANNEL_1
};

/**
 * @brief Configuration struct for ICU on TIM5.
 */
static const ICUConfig icu5cfg = {
	ICU_INPUT_ACTIVE_HIGH,
	200000,   /* 200 kHz ICU clock frequency. */
	icu5widthcb,
	icu5periodcb,
	NULL,
	ICU_CHANNEL_1
};

#endif /* CB_ICU_H */

