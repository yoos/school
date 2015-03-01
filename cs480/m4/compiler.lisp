#!/usr/bin/sbcl --script

(load "lexer")
(load "syntax-parser")
(load "grammar")

(defun my-command-line ()
  (or 
    #+SBCL *posix-argv*  
    #+LISPWORKS system:*line-arguments-list*
    #+CMU extensions:*command-line-words*
    nil))

(defparameter args (cdr (my-command-line)))

(loop for arg in args do
      (cond ((not (string= (subseq arg 0 1) "-"))
             (format T "Parsing ~S:~%" arg)
             (with-open-file (istream arg)
               (syntax-parse (lex istream) *grammar* 0))
             (format T "~%~%"))
            (T NIL))
      )
