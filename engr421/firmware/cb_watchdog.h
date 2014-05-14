#ifndef CB_WATCHDOG_H
#define CB_WATCHDOG_H

#include <ch.h>
#include <cb_config.h>

/*
 * @brief Linear rail watchdog.
 *
 * If accumulated electrical load exceeds LINEAR_RAIL_WATCHDOG_LD, watchdog
 * dies. Watchdog does not revive until cumulative load counts down back to 0.
 *
 * @param dir Linear rail direction.
 * @param dc Linear rail duty cycle.
 */
uint8_t feed_linear_rail_watchdog(uint8_t dir, float dc);

/**
 * @brief Debug output
 *
 * @param buffer Output buffer.
 */
void linear_rail_watchdog_debug_output(uint8_t *buffer);

#endif /* CB_WATCHDOG_H */

