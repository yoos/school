TARGET=cs411_project3_group03_writeup
TARBALL=CS411_project3_group03.tar.bz2

default: tex

tex:
	pdflatex ${TARGET}.tex
	#bibtex ${TARGET}.aux

view: tex
	xpdf ${TARGET}.pdf

tarball: tex
	tar cvjf ${TARBALL} ${TARGET}.tex Makefile output.txt project3.patch

clean:
	rm ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf ${TARBALL}

