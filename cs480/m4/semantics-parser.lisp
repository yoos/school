(load "tokens")

;;; De-terminalize so we can match against production rules.
(defun determ (grammar expr-type)
  (let ((gN (nth 0 grammar))   ; Nonterminals
        (gG (nth 2 grammar))   ; Productions
        (matches (mapcar (lambda (R)
                           (equal (cadr R)
                                  (list expr-type)))
                         (nth 2 grammar))))   ; List of rules that match
    (let ((match-idx (position T matches)))
      (if (and (not (null match-idx))
               (not (member expr-type gN)))
        (car (nth match-idx gG))
        expr-type))
    ))

(defun semantics-parse (parse-tree grammar depth)
  (let ((sentence ())
        (gG (nth 2 grammar)))
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
                                             gG))))
          (if (null match-pos)
            'error
            (car (nth match-pos gG))))))
    ))
