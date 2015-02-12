compiler:
	@echo "Usage: ./lexer.lisp"

stutest.out:
	./lexer.lisp >> stutest.out

proftest.out:
	cat $(PROFTEST)
	compiler $(PROFTEST) > proftest.out
	cat proftest.out

report:
	pdflatex m2.tex
	pdflatex m2.tex

view: report
	xpdf m2.pdf

clean:
	rm -f stutest.out m2.{aux,log,out,pdf}
	ls
