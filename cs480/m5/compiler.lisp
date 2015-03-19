#!/usr/bin/sbcl --script

(load "lexer")
(load "syntax-parser")
(load "semantics-parser")
(load "grammar")
(load "config")

(defun my-command-line ()
  (or 
    #+SBCL *posix-argv*  
    #+LISPWORKS system:*line-arguments-list*
    #+CMU extensions:*command-line-words*
    nil))

(defparameter args (cdr (my-command-line)))

(if (= (length args) 0)
  (format T "Usage:~%    ./compiler.lisp <input file> [-d]~%"))

(loop for arg in args do
      (cond ((string= arg "-d")
             (setf *enable-debug* T))
            (T NIL)))

(loop for arg in args do
      (cond ((not (string= (subseq arg 0 1) "-"))
             (format T "Parsing ~S: " arg)
             (with-open-file (istream arg)
               (let* ((token-list (lex istream))
                      (dummy (format *enable-debug* "~%Token list: ~A~%~%" token-list))
                      (parse-result (parse token-list))
                      (dummy (format *enable-debug* "~%Syntax check: ~A~%~%" parse-result)))

                 ;; Abort on syntax error
                 (if (not parse-result)
                   (format T "Syntax error")
                   (let* ((parse-tree (syntax-parse token-list))
                          (dummy (format *enable-debug* "~%Syntax tree: ~A~%~%" parse-tree))
                          (semantics-result (semantics-parse parse-tree))
                          (dummy (format *enable-debug* "~%Gforth output: ~A~%~%" semantics-result))
                          (dummy (format *enable-debug* "~%")))
                     (format T "~S" parse-result)

                     ;; Write to file, creating directories if necessary and overwriting preexisting files.
                     (with-open-file (ostream (ensure-directories-exist (format NIL "out/~A" arg))
                                              :direction :output
                                              :if-exists :supersede)
                       (format ostream "~A" semantics-result)
                       )))
                 (format T "~%")
                 )))
            (T NIL))
      )
