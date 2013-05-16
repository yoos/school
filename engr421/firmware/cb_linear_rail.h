#ifndef CB_LINEAR_RAIL_H
#define CB_LINEAR_RAIL_H

#include <ch.h>
#include <hal.h>
#include <cb_icu.h>
#include <cb_pid.h>
#include <cb_config.h>
#include <cb_math.h>
#include <cb_linear_rail_controller.h>
#include <chsprintf.h>

/**
 * @brief Set up linear rail.
 */
void setup_linear_rail(void);

/**
 * @brief Update linear rail velocity and stuff.
 *
 * @param dc Array of duty cycles.
 * @param base_wheel_dc Duty cycle of death ray. Used here just to determine
 * enable status. TODO: Do this properly.
 */
void update_linear_rail(float *dc, float base_wheel_dc);

/**
 * @brief Debug output
 *
 * @param buffer Output buffer.
 */
void linear_rail_debug_output(uint8_t *buffer);

#endif /* CB_LINEAR_RAIL_H */

