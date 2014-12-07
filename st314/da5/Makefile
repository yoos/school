TARGET=da5

default: tex

tex:
	pdflatex ${TARGET}.tex
	pdflatex ${TARGET}.tex
	#bibtex ${TARGET}.aux

view: tex
	xpdf ${TARGET}.pdf

clean:
	rm ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf

