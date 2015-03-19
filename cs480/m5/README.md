IBTL to Gforth compiler
=======================
_Compiler written in Common Lisp for CS480_

This compiler is written in ANSI Common Lisp - there is nothing to compile!
Just use:

```
  ./compiler.lisp <input file> [-d]
```

The optional `-d` flag enables debug, which will make the lexer, syntax parser,
and semantic parser print debug info.

Test files are located in `test`, categorized between `good` and `bad` per
their expected parse results. The resultant gforth code will be output to the
`out` directory mirroring the input directory structure.

To run all tests:

```
make stutest.out
```

To run proftest, either run:

```
PROFTEST=/path/to/proftest make proftest.out
```

which will output gforth code to `out/path/to/proftest`, or simply:

```
./compiler.lisp proftest.in
```

which will output gforth to `out/proftest.in`.
