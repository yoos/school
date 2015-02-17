(load "tokens")

;;; Convenience function so popping from symbol table fails gracefully.
(defun table-pop (symbol-table)
  (if (> (fill-pointer symbol-table) 0)
    (vector-pop symbol-table)))

(defun syntax-parse (symbol-table grammar depth)
  (do
    ((sym (table-pop symbol-table)
          (table-pop symbol-table)))
    ((null sym))
    ()
    (let ((token-type (nth 0 sym))
          (token      (nth 1 sym)))
      ;; Handle closing parenthesis specially
      (cond ((and (equal token-type :op-t)
                  (string= token ")"))
             (format T "~,,v,@A~%" (* 2 (- depth 1)) token)
             (return))
            (T NIL))

      ;; Print to screen at correct indentation
      ;; See http://stackoverflow.com/questions/20072959/lisp-format-a-character-a-number-of-times
      (format T "~,,v,@A~%" (* 2 depth) token)

      ;; Recurse on opening parenthesis
      (cond ((and (equal token-type :op-t)
                  (string= token "("))
             (syntax-parse symbol-table grammar (+ 1 depth)))
            (T NIL))
      )))
