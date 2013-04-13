/**
 * @file    chsprintf.h
 * @brief   Mini sprintf-like functionality.
 *
 * Modified from ChibiOS's chprintf.h.
 */

#ifndef _CHSPRINTF_H_
#define _CHSPRINTF_H_

/**
 * @brief   Float type support.
 */
#if !defined(CHPRINTF_USE_FLOAT) || defined(__DOXYGEN__)
#define CHPRINTF_USE_FLOAT          FALSE
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void chsprintf(uint8_t *buf, const char *fmt, ...);
#ifdef __cplusplus
}
#endif

#endif /* _CHSPRINTF_H_ */

