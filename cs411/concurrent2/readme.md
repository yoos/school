CS411 Concurrency Assignment 2
==============================
Spawns a producer and consumer thread. Produces and consumes a number of events
specified by NUM_CONSUME by default, and producer overproduces by
NUM_OVERPRODUCE.

The program will never exit if NUM_OVERPRODUCE is either negative or exceeds
BUFFER_CAPACITY due to the consumer or producer blocking, respectively.

Otherwise, the final buffer size should equal NUM_OVERPRODUCE.

