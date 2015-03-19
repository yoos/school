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
                ((string= ibtl-expr "not") "invert")
                ((string= ibtl-expr "sin") "fsin")
                ((string= ibtl-expr "cos") "fcos")
                ((string= ibtl-expr "tan") "ftan")
                ((equal ibtl-type 'real-ct) (format NIL "~Ae" ibtl-expr))
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

      ;; Catch expressions led by an operator, primitive, or statement before
      ;; we go deeper.
      ((or (typep (caar parse-tree) 'operator-t)
           (typep (caar parse-tree) 'primitive-t)
           (typep (caar parse-tree) 'statement-t))
       (parse-info (format NIL "Op/prim/stmt ~S~%" (car parse-tree)))

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
              (setf return-type (semantics-parse-recurse expr)))
            )

          (parse-info (format NIL "Final return: ~S~%" return-type))
          return-type))
      )))
