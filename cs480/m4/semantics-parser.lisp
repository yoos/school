(load "tokens")

(defun semantics-parse (parse-tree grammar depth)
  (let ((sentence ())
        (gP (nth 2 grammar)))
    (cond
      ;; Base case
      ((typep (car parse-tree) 'token-t)
       (car parse-tree))

      ;; Recurse
      (T
        (do
          ((expr       (car parse-tree) (car parse-tree))
           (parse-tree (cdr parse-tree) (cdr parse-tree)))
          ((null expr))   ; Stop when we can't pop any more
          (let ((expr-type (semantics-parse expr grammar (+ depth 1))))
            (setf sentence (cons expr-type sentence))
            ))

        (setf sentence (nreverse sentence))   ; Reverse sentence
        (format T "s~S: ~S~%" depth sentence)

        ;; Try to match rule
        (let ((match-pos (position T (mapcar (lambda (R)
                                               (equal (cadr R)
                                                      sentence))
                                             gP))))
          (if (null match-pos)
            'error
            (car (nth match-pos gP))))))
    ))
