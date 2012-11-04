TARGET=uniquify
CC = gcc
CXX = g++
CFLAGS = -Wall -std=c99 -openmp -O3 -g -I.
CXXFLAGS = -Wall -openmp -O3 -g
LDFLAGS = -lrt -lpthread 

INCLUDES = 
SOURCE = uniquify.c


default: ${TARGET}

all: ${TARGET} tex

clean:
	rm ${TARGET} ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf


### Program ###

${TARGET}: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS}

debug: ${SOURCE} ${INCLUDES}
	${CC} ${CFLAGS} ${SOURCE} -o ${TARGET} ${LDFLAGS} -DDEBUG


### TeX ###

tex: ${TARGET}.tex 
	pdflatex ${TARGET}.tex
	pdflatex $(TARGET).tex

