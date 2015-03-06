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
    (if (or (typep (car parse-tree) 'token-t)
            (null parse-tree))
      (setf sentence (list (car parse-tree)))
      (let ()
        (setf sentence (list 'leftp-dt))
        (do
          ((expr (car parse-tree) (car parse-tree))
           (parse-tree (cdr parse-tree) (cdr parse-tree))
           (expr-type (semantics-parse (car parse-tree) grammar (+ depth 1))
                      (semantics-parse (car parse-tree) grammar (+ depth 1))))
          ((null expr))
          ()
          (setf sentence (cons (determ grammar expr-type) sentence)))
        (setf sentence (cons 'rightp-dt sentence))))
    (setf sentence (nreverse sentence))
    (format T "s~S: ~S~%" depth sentence)
    (let ((match-pos (position T (mapcar (lambda (R)
                                           (equal (cadr R)
                                                  sentence))
                                         gG))))
      (if (null match-pos)
        'unknown-t
        (car (nth match-pos gG))))
    ))
