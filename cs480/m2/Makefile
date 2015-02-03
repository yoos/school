compiler:
	@echo "Usage: ./lexer.lisp"

stutest.out:
	./lexer.lisp >> stutest.out

proftest.out:
	cat $(PROFTEST)
	compiler $(PROFTEST) > proftest.out
	cat proftest.out

clean:
	rm -f stutest.out
	ls
