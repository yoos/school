#include <buffer.h>

static void add(buffer_t *self, event_t *ev)
{
	pthread_mutex_lock(&self->mutex);
	while (self->capacity == self->size) {
		pthread_cond_wait(&self->cond, &self->mutex);
	}

	if (self->size != 0) {
		self->head = (self->head+1) & (self->capacity-1);
	}
	memcpy(&self->items[self->head], ev, sizeof(event_t));
	self->size++;

	pthread_cond_broadcast(&self->cond);
	pthread_mutex_unlock(&self->mutex);
}

static event_t *get(buffer_t *self)
{
	pthread_mutex_lock(&self->mutex);
	while (self->size == 0) {
		pthread_cond_wait(&self->cond, &self->mutex);
	}

	event_t *ev = &self->items[self->tail];
	if (self->size != 1) {
		self->tail = (self->tail+1) & (self->capacity-1);
	}
	self->size--;

	pthread_cond_broadcast(&self->cond);
	pthread_mutex_unlock(&self->mutex);

	return ev;
}

buffer_t *new_buffer(uint32_t capacity)
{
	buffer_t *buf = (buffer_t*) malloc(sizeof(buffer_t));

	buf->add = (void*) &add;
	buf->get = &get;
	pthread_mutex_init(&buf->mutex, NULL);
	pthread_cond_init(&buf->cond, NULL);

	if (capacity > 0 && capacity <= 65536) {
		buf->capacity = capacity;
		buf->size = 0;
		buf->items = (event_t*) malloc(sizeof(event_t)*capacity);
		buf->head = 0;
		buf->tail = 0;
	}

	return buf;
}

void destroy_buffer(buffer_t *buf)
{
	pthread_mutex_destroy(&buf->mutex);
	pthread_cond_destroy(&buf->cond);
	free(buf->items);
	free(buf);
}

