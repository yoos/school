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
	rm ${TARGET} ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf


### Program ###

${TARGET}: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS}

debug: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS} -DDEBUG


### Plot ###

plot: plot/${TARGET}.gp plot/${TARGET}.t
	cd plot; gnuplot ${TARGET}.gp


### TeX ###

tex: ${TARGET}.tex 
	pdflatex ${TARGET}.tex
	pdflatex $(TARGET).tex

