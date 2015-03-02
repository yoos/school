(load "tokens")

;;; Convenience function so popping from symbol table fails gracefully.
(defun table-pop (symbol-table)
  (if (> (fill-pointer symbol-table) 0)
    (vector-pop symbol-table)))

(defun syntax-parse (symbol-table grammar depth)
  (let ((subtree ()))
    (do
      ((sym (table-pop symbol-table)
            (table-pop symbol-table)))
      ((null sym))
      ()
      (let ((token-type (nth 0 sym))
            (token      (nth 1 sym)))
        (cond ((equal token-type :rightp-dt)   ;; Handle closing parenthesis specially
               (format T "~,,v,@A~%" (* 2 (- depth 1)) token)
               (return))
              ((equal token-type :leftp-dt)   ;; Recurse on opening parenthesis
               (format T "~,,v,@A~%" (* 2 depth) token)
               (setf subtree (cons (syntax-parse symbol-table grammar (+ 1 depth)) subtree)))
              (T
                ;; Print to screen at correct indentation
                ;; See http://stackoverflow.com/questions/20072959/lisp-format-a-character-a-number-of-times
                (format T "~,,v,@A~%" (* 2 depth) token)
                (setf subtree (cons sym subtree))
                ))))
    (nreverse subtree)))
