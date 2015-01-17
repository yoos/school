stutest.out:
	echo -ne "Output 1:\n`gforth 1.fs`\n\n" >> stutest.out
	echo -ne "Output 2:\n`gforth 2.fs`\n\n" >> stutest.out
	echo -ne "Output 3:\n`gforth 3.fs`\n\n" >> stutest.out
	echo -ne "Output 4:\n`gforth 4.fs`\n\n" >> stutest.out
	echo -ne "Output 5:\n`gforth 5.fs`\n\n" >> stutest.out
	echo -ne "Output 6:\n`gforth 6.fs`\n\n" >> stutest.out
	echo -ne "Output 7:\n`gforth 7.fs`\n\n" >> stutest.out
	echo -ne "Output 8:\n`gforth 8.fs`\n\n" >> stutest.out
	echo -ne "Output 9:\n`gforth 9.fs`\n\n" >> stutest.out
	echo -ne "Output 10:\n`gforth 10.fs`\n\n" >> stutest.out
	echo -ne "Output 11:\n`gforth 11.fs`\n\n" >> stutest.out
	echo -ne "Output 12:\n`gforth 12.fs`\n\n" >> stutest.out

proftest.out:
	cat $(PROFTEST)
	compiler $(PROFTEST) > proftest.out
	cat proftest.out

clean:
	rm -f stutest.out
	ls
