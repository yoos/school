TARGET=concurrent2
CFLAGS=-I. -Wall -pthread

all:
	gcc -o ${TARGET} main.c buffer.c event.c producer.c consumer.c ${CFLAGS}

clean:
	rm ${TARGET}

