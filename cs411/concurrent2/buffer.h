#ifndef _BUFFER_H
#define _BUFFER_H

#include <malloc.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>
#include <event.h>
#include <stdio.h>

typedef struct buffer {
	/* Trying to imitate OOP with function pointers.. */
	void (*add)(struct buffer *self, event_t *ev);
	event_t *(*get)(struct buffer *self);

	/* Thread safety */
	pthread_mutex_t mutex;;
	pthread_cond_t cond;

	/* Buffer info */
	uint32_t capacity;
	uint32_t size;
	event_t *items;
	uint32_t head;
	uint32_t tail;
} buffer_t;

/* Constructor */
buffer_t *new_buffer(uint32_t capacity);

/* Destructor */
void destroy_buffer(buffer_t *buf);

#endif /* _BUFFER_H */

