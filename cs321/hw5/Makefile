TARGET=collatz
CC = gcc
CXX = g++
CFLAGS = -Wall -std=c99 -openmp -O3 -g -I.
CXXFLAGS = -Wall -openmp -O3 -g
LDFLAGS = -lrt -lpthread 

INCLUDES = 
SOURCE = ${TARGET}.c


default: ${TARGET}

all: ${TARGET} plot tex

clean:
	rm -f ${TARGET} ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf \
		plot/${TARGET}.png plot/${TARGET}.t


### Program ###

${TARGET}: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS}

debug: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS} -DDEBUG


### Plot ###

plot: ${TARGET} plot/${TARGET}.gp
	if [ ! -f plot/${TARGET}.t ]; \
		then ./${TARGET} 10000 > plot/${TARGET}.t; \
	fi
	cd plot; gnuplot ${TARGET}.gp


### TeX ###

tex: plot ${TARGET}.tex
	pdflatex ${TARGET}.tex
	pdflatex $(TARGET).tex


.PHONY: plot clean

