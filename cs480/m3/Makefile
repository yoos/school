compiler:
	@echo "Usage: ./lexer.lisp"

stutest.out:
	./lexer.lisp >> stutest.out

proftest.out:
	cat $(PROFTEST)
	compiler $(PROFTEST) > proftest.out
	cat proftest.out

report:
	pdflatex m3.tex
	pdflatex m3.tex

view: report
	xpdf m3.pdf

clean:
	rm -f stutest.out m3.{aux,log,out,pdf}
	ls
