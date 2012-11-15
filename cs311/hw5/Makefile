TARGET=primes
CC = gcc
CXX = g++
CFLAGS = -Wall -openmp -O3 -g -I.
CXXFLAGS = -Wall -openmp -O3 -g
LDFLAGS = -lm -lrt -lpthread

INCLUDES =
SOURCE = primes.c


default: ${TARGET}

all: ${TARGET} tex

clean:
	rm -rf ${TARGET} ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf lists


### Program ###

${TARGET}: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS}

debug: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS} -DDEBUG

lists:
	if [ ! -d lists ]; then mkdir lists; fi
	python randgen.py u 1 > lists/arrayUniq1
	python randgen.py u 10 > lists/arrayUniq10
	python randgen.py u 1000 > lists/arrayUniq1000
	python randgen.py u 100000 > lists/arrayUniq100000
	python randgen.py d 1000 > lists/arrayDup1000



### TeX ###

tex: ${TARGET}.tex 
	pdflatex ${TARGET}.tex
	pdflatex $(TARGET).tex


.PHONY: clean lists
