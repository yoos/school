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

#define MODE_POS 0
#define MODE_VEL 1

/**
 * @brief Set up linear rail.
 */
void setup_linear_rail(void);

/**
 * @brief Calibrate linear rail.
 *
 */
uint8_t calibrate_linear_rail(uint8_t status, uint8_t limit_switch, uint8_t *dir, float *dc);

/**
 * @brief Update linear rail velocity and stuff.
 *
 * @param status Game status
 * @param mode Control mode, either position (0) or velocity (1)
 * @param target Target value
 *
 * @output dir Direction of movement (0 or 1).
 * @output dc Duty cycle for linear rail motor.
 */
void update_linear_rail(uint8_t status, uint8_t mode, float target, uint8_t *dir, float *dc);

/**
 * @brief Debug output
 *
 * @param buffer Output buffer.
 */
void linear_rail_debug_output(uint8_t *buffer);

#endif /* CB_LINEAR_RAIL_H */

