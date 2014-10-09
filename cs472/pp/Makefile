TARGET=pp
TARBALL=yoos_pp.tar.bz2

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

