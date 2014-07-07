#ifndef _PRODUCER_H
#define _PRODUCER_H

#include <pthread.h>
#include <buffer.h>
#include <event.h>

void producer(buffer_t *buf);

#endif /* _PRODUCER_H */

