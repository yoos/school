compiler:
	@echo "Usage: ./compiler.lisp <input file>"

stutest.out:
	./compiler.lisp test/good/*.txt test/bad/*.txt >> stutest.out

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
	rm -rf out
	ls
