#include <event.h>

event_t gen_random_event(void)
{
	event_t event = 0;
	while (event == 0) {
		event = rand();
	}

	return event;
}

event_t get_keyboard_event(void)
{
	/* TODO(yoos) */
	event_t event = 117;

	return event;
}

event_t process(event_t *ev)
{
	/* TODO(yoos): This is temporary. */
	if (ev == 0) {
		return 0;
	}
	return *ev;
}

