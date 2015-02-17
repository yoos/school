#!/usr/bin/sbcl --script

(load "lexer")
(load "parser")

(with-open-file (istream "proftest.in")
  (parse (lex istream)))
