compiler:
	@echo "Usage: ./lexer.lisp"

stutest.out:
	./compiler.lisp inputs/2.txt inputs/3.txt >> stutest.out

proftest.out:
	cat $(PROFTEST)
	./compiler.lisp $(PROFTEST) > proftest.out
	cat proftest.out

report:
	pdflatex m4.tex
	pdflatex m4.tex

view: report
	xpdf m4.pdf

clean:
	rm -f stutest.out m4.{aux,log,out,pdf}
	ls
