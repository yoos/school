#ifndef CB_DEATH_RAY_H
#define CB_DEATH_RAY_H

#include <ch.h>
#include <hal.h>
#include <cb_icu.h>
#include <cb_pid.h>
#include <cb_config.h>
#include <cb_math.h>
#include <chsprintf.h>

/**
 * @brief Set up death ray.
 */
void setup_death_ray(void);

/**
 * @brief Update death ray velocity and stuff.
 *
 * @param dc Array of duty cycles.
 */
void update_death_ray(float *dc, float base_wheel_dc);

/**
 * @brief Debug output
 *
 * @param buffer Output buffer.
 */
void death_ray_debug_output(uint8_t *buffer, float base_wheel_dc);

#endif /* CB_DEATH_RAY_H */
