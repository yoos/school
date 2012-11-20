TARGET=primes
CC = gcc
CXX = g++
CFLAGS = -Wall -openmp -O3 -g -I.
CXXFLAGS = -Wall -openmp -O3 -g
LDFLAGS = -lm -lrt -lpthread

INCLUDES = num_primes.h
SOURCE = primes.c


default: ${TARGET}

all: ${TARGET} tex

clean:
	rm -rf ${TARGET} ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf


### Program ###

${TARGET}: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS}

debug: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS} -DDEBUG


### TeX ###

tex: ${TARGET}.tex 
	pdflatex ${TARGET}.tex
	pdflatex $(TARGET).tex


.PHONY: clean
