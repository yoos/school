PROJ=p3
TEX=p3

all:
	shaderdl $(PROJ)s.sl $(PROJ)d.sl
	renderdl $(PROJ).rib

tex:
	pdflatex $(TEX).tex
	pdflatex $(TEX).tex

view: tex
	xpdf $(TEX).pdf

clean:
	rm $(PROJ)s.sdl $(PROJ)d.sdl $(PROJ).tiff
	rm $(TEX).{aux,log,out,pdf}

