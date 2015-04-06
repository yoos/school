PROJ=a1
TEX=a1

all: tex

tex:
	pdflatex $(TEX).tex
	pdflatex $(TEX).tex

view: tex
	xpdf $(TEX).pdf

clean:
	rm $(TEX).{aux,log,out,pdf}

