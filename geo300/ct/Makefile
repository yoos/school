TEX=ct

all: tex

tex:
	pdflatex $(TEX).tex
	bibtex $(TEX)
	pdflatex $(TEX).tex
	pdflatex $(TEX).tex

view: tex
	xpdf $(TEX).pdf

clean:
	rm $(TEX).{aux,log,out,bbl,blg,pdf}

