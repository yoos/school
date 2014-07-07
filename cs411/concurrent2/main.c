#include <buffer.h>
#include <producer.h>
#include <consumer.h>
#include <stdlib.h>
#include <pthread.h>
#include <time.h>

#define BUFFER_CAPACITY 16   /* Must be power of 2 due to buffer implementation */
#define NUM_CONSUME 100
#define NUM_OVERPRODUCE 10

buffer_t *b;
pthread_t tid[2];

static void *producer_thread(void *arg)
{
	printf("Producer thread up.\n");

	int i;
	for (i=0; i<NUM_CONSUME+NUM_OVERPRODUCE; i++) {
		producer(b);
		printf("[P %3d] event: %5d  Buffer size: %2d/%2d  H: %2d  T: %2d\n", i, b->items[b->head], b->size, b->capacity, b->head, b->tail);
	}
	return NULL;
}

static void *consumer_thread(void *arg)
{
	printf("Consumer thread up.\n");

	int i;
	for (i=0; i<NUM_CONSUME; i++) {
		event_t res = consumer(b);
		printf("[C %3d] event: %5d  Buffer size: %2d/%2d  H: %2d  T: %2d\n", i, res, b->size, b->capacity, b->head, b->tail);
	}
	return NULL;
}

int main(void)
{
	b = new_buffer(BUFFER_CAPACITY);
	printf("Created buffer of capacity %d, current size %d\n\n", b->capacity, b->size);

	/* Seed for random event generator */
	srand(time(NULL));

	pthread_create(&tid[0], NULL, &producer_thread, b);
	pthread_create(&tid[1], NULL, &consumer_thread, b);

	pthread_join(tid[0], NULL);
	pthread_join(tid[1], NULL);

	printf("\nProduced %d and consumed %d.\n", NUM_CONSUME+NUM_OVERPRODUCE, NUM_CONSUME);
	printf("Final buffer size: %d/%d.\n", b->size, b->capacity);

	destroy_buffer(b);

	return 0;
}

