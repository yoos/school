TARGET=cs411_004_project4_individual_writeup
TARBALL=CS411_project4_writeup.tar.bz2

default: tex

tex:
	pdflatex ${TARGET}.tex
	pdflatex ${TARGET}.tex
	#bibtex ${TARGET}.aux

view: tex
	xpdf ${TARGET}.pdf

tarball: tex
	tar cvjf ${TARBALL} ${TARGET}.tex Makefile

clean:
	rm ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf ${TARBALL}

