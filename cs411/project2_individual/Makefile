TARGET=cs411_002_project2_individual_writeup
TARBALL=CS411_project2_writeup.tar.bz2

default: tex

tex:
	pdflatex ${TARGET}.tex
	#bibtex ${TARGET}.aux

view: tex
	xpdf ${TARGET}.pdf

tarball: tex
	tar cvjf ${TARBALL} ${TARGET}.tex Makefile

clean:
	rm ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf ${TARBALL}

