TEX=p2

all:
	shaderdl p2s.sl p2d.sl
	renderdl p2.rib

tex:
	pdflatex $(TEX).tex
	pdflatex $(TEX).tex

view: tex
	xpdf $(TEX).pdf

clean:
	rm p2s.sdl p2d.sdl p2.tiff
	rm $(TEX).{aux,log,out,pdf}

