/*******************************************************
 * sem_fifo - First In First Out Semaphore
 * Concurrent Assignment 1 - CS 411
 * Group 03
 * 
 * Hugh McDonald
 * Bronson Mock
 * Soohyun Yoo
 *
 * 4.11.14
 *******************************************************/
#define _POSIX_C_SOURCE 200809L
#define _BSD_SOURCE

#include <unistd.h>
#include <pthread.h>

typedef struct sem_fifo {
	pthread_cond_t cond;
	pthread_mutex_t mutex;
	unsigned long head;
	unsigned long tail;
} sem_fifo_t;

#define SEM_FIFO_INITIALIZER { PTHREAD_COND_INITIALIZER, PTHREAD_MUTEX_INITIALIZER, 0, 0 }

void wait(sem_fifo_t *queue)
{
	unsigned long position;

	pthread_mutex_lock(&queue->mutex);
	position = queue->tail++;
	while (position != queue->head) {
		pthread_cond_wait(&queue->cond, &queue->mutex);
	}
	pthread_mutex_unlock(&queue->mutex);
}

void signal(sem_fifo_t *queue)
{
	pthread_mutex_lock(&queue->mutex);
	queue->head++;
	pthread_cond_broadcast(&queue->cond);
	pthread_mutex_unlock(&queue->mutex