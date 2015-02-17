#!/usr/bin/sbcl --script

(load "lexer")
(load "syntax-parser")
(load "grammar")

(with-open-file (istream "proftest.in")
  (syntax-parse (lex istream) *grammar*))
