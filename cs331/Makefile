TARGET=wa4

all:
	pdflatex $(TARGET).tex
	pdflatex $(TARGET).tex

view: all
	xpdf $(TARGET).pdf

clean:
	rm $(TARGET).{aux,log,out,pdf}

