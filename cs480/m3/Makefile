compiler:
	@echo "Usage: ./lexer.lisp"

stutest.out:
	./compiler.lisp inputs/2.txt inputs/3.txt >> stutest.out

proftest.out:
	cat $(PROFTEST)
	./compiler.lisp $(PROFTEST) > proftest.out
	cat proftest.out

report:
	pdflatex m3.tex
	pdflatex m3.tex

view: report
	xpdf m3.pdf

clean:
	rm -f stutest.out m3.{aux,log,out,pdf}
	ls
