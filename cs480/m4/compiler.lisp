#!/usr/bin/sbcl --script

(load "lexer")
(load "syntax-parser")
(load "semantics-parser")
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
             (format T "Parsing ~S:~%~%" arg)
             (with-open-file (istream arg)
               (let* ((lexer-list       (lex istream))
                      (syntax-tree      (syntax-parse lexer-list *grammar* 0))
                      (semantics-result (semantics-parse syntax-tree *grammar* 0)))
                 (format T "~%Syntax tree: ~S~%" syntax-tree)
                 (format T "~%Semantics result: ~S~%" semantics-result))
               (format T "~%~%")))
            (T NIL))
      )
