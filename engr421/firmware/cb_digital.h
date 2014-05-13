#ifndef CB_DIGITAL_H
#define CB_DIGITAL_H

#include <ch.h>
#include <hal.h>
#include <cb_config.h>

void setup_digital(void);

/*
 *
 * It's probably really bad that I'm trying to use state for both input and output.
 *
 * @output state
 */
void update_digital(uint8_t *state);

#endif /* CB_DIGITAL_H */

