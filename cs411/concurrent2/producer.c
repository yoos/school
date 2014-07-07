#include <producer.h>

void producer(buffer_t *buf)
{
	event_t ev = gen_random_event();
	buf->add(buf, &ev);
}

