#!/usr/bin/sbcl --script

(load "lexer")
(load "parser")
(load "grammar")

(with-open-file (istream "proftest.in")
  (parse (lex istream) *grammar*))
