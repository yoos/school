#ifndef CB_WATCHDOG_H
#define CB_WATCHDOG_H

#include <ch.h>
#include <cb_config.h>

/*
 * @brief Linear rail watchdog.
 *
 * If accumulated electrical load exceeds LINEAR_RAIL_WATCHDOG_LD, watchdog
 * dies.
 *
 * @param dc Linear rail duty cycle.
 */
uint8_t feed_linear_rail_watchdog(float dc);

#endif /* CB_WATCHDOG_H */

