#ifndef _CONSUMER_H
#define _CONSUMER_H

#include <pthread.h>
#include <buffer.h>
#include <event.h>

event_t consumer(buffer_t *buf);

#endif /* _CONSUMER_H */

