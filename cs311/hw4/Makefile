TARGET=uniquify
CC = gcc
CXX = g++
CFLAGS = -Wall -openmp -O3 -g -I.
CXXFLAGS = -Wall -openmp -O3 -g
LDFLAGS = -lrt -lpthread

INCLUDES = parse.h merge.h usage.h
SOURCE = uniquify.c


default: ${TARGET}

all: ${TARGET} tex

clean:
	rm -f ${TARGET} ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf


### Program ###

${TARGET}: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS}

debug: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS} -DDEBUG

lists:
	if [ ! -d lists ]; then mkdir lists; fi
	python randgen.py 1 > lists/array1
	python randgen.py 10 > lists/array10
	python randgen.py 1000 > lists/array1000
	python randgen.py 100000 > lists/array100000


### TeX ###

tex: ${TARGET}.tex 
	pdflatex ${TARGET}.tex
	pdflatex $(TARGET).tex


.PHONY: clean lists
