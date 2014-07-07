#ifndef _EVENT_H
#define _EVENT_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

/* Dummy event type */
typedef uint16_t event_t;

event_t gen_random_event(void);
event_t get_keyboard_event(void);   /* TODO(yoos) */
event_t process(event_t *ev);

#endif /* _EVENT_H */

