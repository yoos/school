#include <consumer.h>

event_t consumer(buffer_t *buf)
{
	event_t *ev = buf->get(buf);
	return process(ev);
}

