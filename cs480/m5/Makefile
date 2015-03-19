compiler:
	@echo "Usage: ./compiler.lisp <input file> [-d]"

stutest.out:
	./compiler.lisp test/good/*.ibtl test/bad/*.ibtl >> stutest.out

proftest.out:
	cat $(PROFTEST)
	./compiler.lisp $(PROFTEST) > proftest.out
	cat proftest.out

report:
	pdflatex m5.tex
	pdflatex m5.tex

view: report
	xpdf m5.pdf

clean:
	rm -f stutest.out m5.{aux,log,out,pdf}
	rm -rf out
	ls
