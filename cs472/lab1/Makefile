TARGET=lab1

default: tex

tex:
	pdflatex ${TARGET}.tex
	pdflatex ${TARGET}.tex
	#bibtex ${TARGET}.aux

bench:
	clang -o fpbench main.c -Wall -Wextra

view: tex
	xpdf ${TARGET}.pdf

clean:
	rm ${TARGET}.aux ${TARGET}.log ${TARGET}.out ${TARGET}.pdf fpbench

