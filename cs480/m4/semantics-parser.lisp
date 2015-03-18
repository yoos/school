(load "tokens")
(load "util")
(load "config")

(defparameter *gforth-stack* ())
(defparameter *depth* 0)   ; TODO: The syntax parser initializes this on its own. We shouldn't use global vars.

;;; Add gforth expression to global stack.
(defun add-gforth (expr)
  (parse-info (format NIL "Adding ~S to stack~%" expr))
  (setf *gforth-stack* (cons expr *gforth-stack*)))

;;; Translate some differences between IBTL and Forth.
(defun ibtl-to-gforth (sym)
  (let ((gforth-expr (cdr sym)))
    (setf gforth-expr
          (cond ((string= gforth-expr "!=")  "<>")
                ((string= gforth-expr "%")   "mod")
                ((string= gforth-expr "^")   "**")
                ((string= gforth-expr "not") "invert")
                (T gforth-expr)))
    (add-gforth gforth-expr)   ; Add to stack. TODO: maybe I shouldn't do this here
    (setf (cdr sym) gforth-expr))
  sym)

;;; Cast integer to float (if not already)
;;; TODO: add safeties against other types?
(defun itof (sym)
  (if (equal (car sym) 'real-ct)
    sym
    (format NIL "~Se" (car sym))))

;;; Top parser call
(defun semantics-parse (parse-tree)
  (format T "Parsing semantics:~%")
  (semantics-parse-recurse parse-tree)

  ;; Neat formatting trick: http://stackoverflow.com/questions/8830888/whats-the-canonical-way-to-join-strings-in-a-list
  (format NIL "~{~A~^ ~}" (nreverse *gforth-stack*)))

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
             (return-type (cons 'unknown "gforth")))
         (do ((expr          (car operands-tree) (car operands-tree))
              (operands-tree (cdr operands-tree) (cdr operands-tree)))
           ((null expr))   ;; Stop when we can't pop any more
           (let ((operand (semantics-parse-recurse expr)))
             (setf operands (cons operand operands))
             ;; Update return-type depending on opriment
             (case (car opriment)
               (('binop-ot 'unop-ot)
                (setf (cdr return-type) (cdr opriment))
                ;; TODO: process operands and adjust return type accordingly
                (setf (car return-type)
                      (cond ((strmatch? (cdr opriment) '("and" "or" "not" ">=" "<=" "!="))
                             'boolean-ct)
                            ((strmatch? (cdr opriment) '("sin" "cos" "tan"))
                             'real-ct)
                            (T 'integer-ct)
                            )))
               )))

         ;; Finally, convert opriment to gforth and add to stack.
         (setf opriment (semantics-parse-recurse opriment))
         (setf operands (cons opriment operands))
         ;(setf operands (nreverse operands))

         ;(do ((g

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
