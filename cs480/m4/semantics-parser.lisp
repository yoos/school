(load "tokens")
(load "config")

(defun semantics-parse (parse-tree)
  (format T "Parsing semantics:~%")
  (semantics-parse-recurse parse-tree 0))

(defun semantics-parse-recurse (parse-tree depth)
  (cond
    ;; EOF
    ((equal (car parse-tree) 'eof)
     NIL)

    ;; Base case (raw symbol)
    ((typep (car parse-tree) 'token-t)
     (car parse-tree))

    ;; Recurse (list of symbols)
    (T (let ((sentence ())
             (return-type NIL))
         (do
           ((expr       (car parse-tree) (car parse-tree))
            (parse-tree (cdr parse-tree) (cdr parse-tree)))
           ((null expr))   ; Stop when we can't pop any more
           (let ((expr-type (semantics-parse-recurse expr (+ depth 1))))
             (setf sentence (cons expr-type sentence))
             ))

         (setf sentence (nreverse sentence))   ; Reverse sentence
         (format T "s~S: ~S~%" depth sentence))))
  )
