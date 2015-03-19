(load "tokens")
(load "util")
(load "config")

(defparameter *gforth-stack* ())
(defparameter *depth* 0)   ; TODO: The syntax parser initializes this on its own. We shouldn't use global vars.

;;; Add gforth expressions from symbol list to global stack.
(defun add-gforth (gforth-list)
  (parse-info (format NIL "Adding ~S to stack~%" gforth-list))
  (setf *gforth-stack* (append gforth-list *gforth-stack*)))

;;; Translate some differences between IBTL and Forth.
(defun ibtl-to-gforth (sym)
  (let ((ibtl-type (car sym))
        (ibtl-expr (cdr sym)))
    (setf (cdr sym)
          (cond ((string= ibtl-expr "!=")  "<>")
                ((string= ibtl-expr "%")   "mod")
                ((string= ibtl-expr "^")   "**")
                ((string= ibtl-expr ":=")  "!")
                ((string= ibtl-expr "not") "invert")
                ((string= ibtl-expr "sin") "fsin")
                ((string= ibtl-expr "cos") "fcos")
                ((string= ibtl-expr "tan") "ftan")
                ((string= ibtl-expr "let") "create")
                ((string= ibtl-expr "stdout") ".")
                ((equal ibtl-type 'real-ct) (format NIL "~Ae" ibtl-expr))
                ((equal ibtl-type 'string-ct) (format NIL "s\" ~A\"" ibtl-expr))
                (T ibtl-expr)))
    )
  sym)

;;; Convert gforth symbol to real type
(defun itof (sym-gforth)
  (parse-info (format NIL "Symbol: ~S~%" sym-gforth))
  (cond ((equal (car sym-gforth) 'integer-ct)
         (cons 'real-ct (format NIL "~Ae" (cdr sym-gforth))))
        ((or (equal (car sym-gforth) 'binop-ot)
             (equal (car sym-gforth) 'unop-ot))
         (cond ((not (strmatch? (cdr sym-gforth) '("and" "or" "not")))
                (cons (car sym-gforth) (format NIL "f~A" (cdr sym-gforth))))   ; Prepend with f
               (T sym-gforth)))
        (T sym-gforth)
        ))

;;; Top parser call
(defun semantics-parse (parse-tree)
  (setf *gforth-stack* ())   ; Reset stack
  (parse-info "Parsing semantics:~%")
  (let* ((gforth-result (semantics-parse-recurse parse-tree))
         (gforth-type (car gforth-result))
         (gforth-tree (cdr gforth-result)))
    ;; Neat formatting trick: http://stackoverflow.com/questions/8830888/whats-the-canonical-way-to-join-strings-in-a-list
    (format NIL "~{~A~^ ~}" (flatten gforth-tree))))

;;; Recursive parser call
(defun semantics-parse-recurse (parse-tree)
  (let ((*depth* (+ *depth* 1)))
    (cond
      ;; EOF
      ((equal (car parse-tree) 'eof)
       (parse-info "EOF~%")
       parse-tree)

      ;; Base case (bare symbol)
      ((typep (car parse-tree) 'token-t)
       (parse-info (format NIL "Bare symbol ~S~%" parse-tree))
       (ibtl-to-gforth parse-tree))

      ;; Catch statements
      ((typep (caar parse-tree) 'statement-t)
       (parse-info (format NIL "Stmt ~S~%" (car parse-tree)))
       (let ((stmt          (car parse-tree))
             (operands-tree (cdr parse-tree))
             (operands ())
             (return-type (cons 'unknown "gforth")))
         (case (car stmt)
           ('if-st
            (let ((if-cond  (nth 0 operands-tree))
                  (if-true  (nth 1 operands-tree))
                  (if-false (nth 2 operands-tree)))
              ;; Manually stick some gforth on the stack..
              ;; TODO: This is icky.
              (setf operands (cons (semantics-parse-recurse if-cond) operands))
              (setf operands (cons (cons 'if-st "if") operands))
              (setf operands (cons (semantics-parse-recurse if-true) operands))
              (cond ((not (null if-false))
                     (setf operands (cons (cons 'ifelse-st "else") operands))
                     (setf operands (cons (semantics-parse-recurse if-false) operands))))
              (setf operands (cons (cons 'ifthen-st "then") operands)))
            (setf (car return-type) 'integer-ct))   ; TODO: What should I return here?
           ('while-st
            (let ((while-cond (car operands-tree))
                  (while-exprs (cdr operands-tree)))
              (setf operands (cons (cons 'whilebegin-st "begin") operands))
              (setf operands (cons (semantics-parse-recurse while-cond) operands))
              (setf operands (cons (cons 'whilebegin-st "while") operands))
              (setf operands (cons (semantics-parse-recurse while-exprs) operands))
              (setf operands (cons (cons 'whilebegin-st "repeat") operands)))
            (setf (car return-type) 'integer-ct))
           ('let-st
            (let ((let-exprs (car operands-tree)))
              (parse-info (format NIL "~S~%" let-exprs))
              (do ((id        (car let-exprs) (car let-exprs))
                   (let-exprs (cdr let-exprs) (cdr let-exprs)))
                ((null id))
                (setf operands (cons (cons 'letcreate-st "create") operands))
                (setf operands (cons (car id) operands))   ; Variable name
                ; TODO: Should we do anything with the variable type?
                ))
            (setf (car return-type) 'integer-ct))
           ('stdout-st
            (setf operands (cons (semantics-parse-recurse operands-tree) operands))
            (setf operands (cons (cons 'stdoutpad-st "pad place") operands))
            (setf operands (cons (cons 'stdoutpad-st "pad count type") operands))
            (setf (car return-type) 'integer-ct))
           ('assign-st
            (let ((assign-id  (nth 0 operands-tree))
                  (assign-val (nth 1 operands-tree)))
              (setf operands (cons (semantics-parse-recurse assign-val) operands))
              (setf operands (cons assign-id operands))
              (if (equal (car return-type) 'real-ct)
                (setf operands (cons (cons 'assign-st "f!") operands))
                (setf operands (cons (cons 'assign-st "!") operands))))
            (setf (car return-type) 'integer-ct)))

         (setf operands (nreverse operands))

         ;; Grab only the gforth code, as we no longer need operand types.
         (setf operands (mapcar #'cdr operands))
         (setf (cdr return-type) operands)

         (parse-info (format NIL "Operands: ~S~%" operands))
         (parse-info (format NIL "Return: ~S~%" return-type))
         return-type))

      ;; Catch expressions led by an operator, primitive, or statement before
      ;; we go deeper.
      ((or (typep (caar parse-tree) 'operator-t)
           (typep (caar parse-tree) 'primitive-t))
           ;(typep (caar parse-tree) 'statement-t))
       (parse-info (format NIL "Op/prim ~S~%" (car parse-tree)))

       ;; Collect operands and stick the operator/primitive/statement
       ;; (opriment?) at the end.
       (let ((opriment      (car parse-tree))
             (operands-tree (cdr parse-tree))
             (operands ())
             (return-type (cons 'unknown "gforth")))   ; TODO: Replace with something less arbitrary
         ;; Process opriment and set it aside for now
         (setf (car return-type)
               (case (car opriment)
                 (('binop-ot 'unop-ot)
                  (cond ((strmatch? (cdr opriment) '("and" "or" "not" ">=" "<=" "!="))
                         'boolean-ct)
                        ((strmatch? (cdr opriment) '("sin" "cos" "tan"))
                         'real-ct)
                        (T 'integer-ct)))
                 ))

         ;; Process operands
         (do ((expr          (car operands-tree) (car operands-tree))
              (operands-tree (cdr operands-tree) (cdr operands-tree)))
           ((null expr))   ;; Stop when we can't pop any more
           (let ((operand (semantics-parse-recurse expr)))
             (setf operands (cons operand operands))   ; Add to list
             ;; Update return-type depending on operand
             (case (car return-type)
               ('integer-ct
                (case (car operand)
                  ('real-ct (setf (car return-type) 'real-ct))))
               (otherwise
                 (case (car operand)
                   ('string-ct (setf (car return-type) 'string-ct))
                   (otherwise (setf (car return-type) 'integer-ct)))))
             ))

         ;; Finally, convert opriment to gforth and add to local symbol stack.
         (setf opriment (semantics-parse-recurse opriment))
         (setf operands (cons opriment operands))
         (setf operands (nreverse operands))

         ;; If return-type is real-ct at this point, blanket convert everything
         ;; to reals types, including the operator.
         (if (equal (car return-type) 'real-ct)
           (setf operands (mapcar #'itof operands)))

         ;; Grab only the gforth code, as we no longer need operand types.
         (setf operands (mapcar #'cdr operands))
         (setf (cdr return-type) operands)

         (parse-info (format NIL "Operands: ~S~%" operands))
         (parse-info (format NIL "Return: ~S~%" return-type))
         return-type))

      ;; Iterate over expressions and return the type of the last expression,
      ;; excluding EOF.
      (T
        ;; Loop through all expressions
        (let ((return-type NIL))
          (do ((expr       (car parse-tree) (car parse-tree))
               (parse-tree (cdr parse-tree) (cdr parse-tree)))
            ((null expr))   ; Stop when we can't pop any more
            (if (not (equal (car expr) 'eof))
              (let ((return-old (cdr return-type))
                    (return-new (semantics-parse-recurse expr)))
                (setf return-type (cons (car return-new)
                                        (append return-old (cdr return-new))
                                        )))
              ))

          (parse-info (format NIL "Final return: ~S~%" return-type))
          return-type))
      )))
