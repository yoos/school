PROJ=p5
TEX=p5

all: tex

tex:
	pdflatex $(TEX).tex
	pdflatex $(TEX).tex

view: tex
	xpdf $(TEX).pdf

clean:
	rm $(TEX).{aux,log,out,pdf}

