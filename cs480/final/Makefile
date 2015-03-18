all: report

report:
	pdflatex final.tex
	bibtex final
	pdflatex final.tex
	pdflatex final.tex

view: report
	xpdf final.pdf

clean:
	rm -f final.{bbl,blg,aux,log,out,pdf}
	ls
