(load "tokens")
(load "util")
(load "config")

(defparameter *sym* ())
(defparameter *symbol-table* ())
(defparameter *accept* T)
(defparameter *idx* 0)

(defun init-globals (token-list)
  (setf *sym* ()
        *symbol-table* token-list
        *accept* T
        *depth* 0
        *idx* 0
        ))

;;; Convenience function so popping from symbol table fails gracefully.
(defun table-pop (token-list)
  (let ((token (car token-list))
        (table (cdr token-list)))
    (setf *symbol-table* table)
    token))

(defun get-sym ()
  (setf *sym* (table-pop *symbol-table*)))

(defun try-reject (expect production)
  (cond (expect
          (parse-reject (format NIL "Parse failed on ~@?~%" production))
          (setf *accept* NIL)
          T)
        (T NIL)))

(defun peek (token-type)
  (equal (car *sym*) token-type))

(defun accept (token-type)
  (let ((*depth* (+ *depth* 1)))
    ;(parse-info (format NIL "Symbol to accept is ~S.. " *sym*))
    (cond ((equal (car *sym*) token-type)
           (parse-info (format NIL "Accepting ~S. Next is ~S~%" token-type (car (get-sym))))
           T)
          (T
            ;(parse-info (format NIL "rejecting ~S~%" token-type) NIL)
            NIL)))
  )

(defun expect (token-type)
  (cond ((accept token-type) T)
        (T
          (parse-reject (format NIL "Expected ~S, got ~S instead~%" token-type (car *sym*))) NIL)))

(defun parse (token-list)
  (init-globals token-list)
  (get-sym)
  (cond ((and (parse-info "Attempting parse..~%")
              (parse-S)
              (expect 'eof))
         (parse-info "accepted~%") T)
        (T (try-reject T "FILE") NIL))
  *accept*)

(defun parse-S (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying S -> EPSILON..~%")
              (or (peek 'rightp-dt)
                  (peek 'eof)))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying S -> Sexpr S..~%")
                (parse-Sexpr)
                (parse-S T))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying S -> ( PS ) S..~%")
                (accept 'leftp-dt)
                (parse-PS T)
                (expect 'rightp-dt)
                (parse-S T))
           (parse-info "accepted~%") T)
          (T (try-reject expect "S") NIL))))

(defun parse-Sexpr (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying Sexpr -> Soper..~%")
                (parse-Soper))
           (parse-info "accepted~%") T)
          (T (try-reject expect "Sexpr") NIL))))

(defun parse-PS (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying PS -> EPSILON..~%")
                (peek 'rightp-dt))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying PS -> Pexpr..~%")
                (parse-Pexpr))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying PS -> S..~%")
                (parse-S))
           (parse-info "accepted~%") T)
          (T (try-reject expect "PS") NIL))))

(defun parse-Soper (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying Soper -> const..~%")
                (parse-const))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Soper -> id..~%")
                (parse-id))
           (parse-info "accepted~%") T)
          (T (try-reject expect "Soper") NIL))))

(defun parse-Pexpr (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying Pexpr -> Poper..~%")
                (parse-Poper))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Pexpr -> Pstmt..~%")
                (parse-Pstmt))
           (parse-info "accepted~%") T)
          (T (try-reject expect "Pexpr") NIL))))

(defun parse-Poper (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying Poper -> := id oper..~%")
                (accept 'assign-st)
                (parse-id T)
                (parse-oper T))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Poper -> binop oper oper..~%")
                (accept 'binop-ot)
                (parse-oper T)
                (parse-oper T))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Poper -> unop oper..~%")
                (accept 'unop-ot)
                (parse-oper T))
           (parse-info "accepted~%") T)
          (T (try-reject expect "Poper") NIL))))

(defun parse-Pstmt (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying Pstmt -> if expr expr else..~%")
                (accept 'if-st)
                (parse-expr T)
                (parse-expr T)
                (parse-else T))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Pstmt -> while expr exprs..~%")
                (accept 'while-st)
                (parse-expr T)
                (parse-exprs T))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Pstmt -> let ( ids )..~%")
                (accept 'let-st)
                (expect 'leftp-dt)
                (parse-ids T)
                (expect 'rightp-dt))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Pstmt -> stdout oper..~%")
                (accept 'stdout-st)
                (parse-oper T))
           (parse-info "accepted~%") T)
          (T (try-reject expect "Pstmt") NIL))))

(defun parse-oper (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying oper -> Soper..~%")
                (parse-Soper))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying oper -> ( Poper )..~%")
                (accept 'leftp-dt)
                (parse-Poper T)
                (expect 'rightp-dt))
           (parse-info "accepted~%") T)
          (T (try-reject expect "oper") NIL))))

(defun parse-expr (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying expr -> Sexpr..~%")
                (parse-Sexpr))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying expr -> ( Pexpr )..~%")
                (accept 'leftp-dt)
                (parse-Pexpr T)
                (expect 'rightp-dt))
           (parse-info "accepted~%") T)
          (T (try-reject expect "expr") NIL))))

(defun parse-else (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying else -> EPSILON..~%")
                (peek 'rightp-dt))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying else -> expr..~%")
                (parse-expr))
           (parse-info "accepted~%") T)
          (T (try-reject expect "else") NIL))))

(defun parse-exprs (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying exprs -> EPSILON..~%")
                (peek 'rightp-dt))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying exprs -> expr exprs..~%")
                (parse-expr)
                (parse-exprs T))
           (parse-info "accepted~%") T)
          (T (try-reject expect "exprs") NIL))))

(defun parse-ids (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((and (parse-info "Trying ids -> EPSILON..~%")
                (peek 'rightp-dt))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying ids -> ( id prim ) ids..~%")
                (accept 'leftp-dt)
                (parse-id T)
                (parse-prim T)
                (expect 'rightp-dt)
                (parse-ids T))
           (parse-info "accepted~%") T)
          (T (try-reject expect "ids") NIL))))

(defun parse-const (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((accept 'boolean-ct)
           (parse-info "Parsed: const -> boolean-ct~%") T)
          ((accept 'integer-ct)
           (parse-info "Parsed: const -> integer-ct~%") T)
          ((accept 'real-ct)
           (parse-info "Parsed: const -> real-ct~%") T)
          ((accept 'string-ct)
           (parse-info "Parsed: const -> string-ct~%") T)
          (T (try-reject expect "const") NIL))))

(defun parse-id (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((accept 'function-it)
           (parse-info "Parsed: id -> function-ct~%") T)
          ((accept 'variable-it)
           (parse-info "Parsed: id -> variable-ct~%") T)
          (T (try-reject expect "id") NIL))))

(defun parse-prim (&optional (expect NIL))
  (let ((*depth* (+ *depth* 1)))
    (cond ((accept 'boolean-pt)
           (parse-info "Parsed: prim -> boolean-pt~%") T)
          ((accept 'integer-pt)
           (parse-info "Parsed: prim -> integer-pt~%") T)
          ((accept 'real-pt)
           (parse-info "Parsed: prim -> real-pt~%") T)
          ((accept 'string-pt)
           (parse-info "Parsed: prim -> string-pt~%") T)
          (T (try-reject expect "prim") NIL))))



(defun syntax-parse (token-list)
  (init-globals token-list)
  (parse-info "Producing syntax tree:~%")
  (syntax-parse-recurse))

(defun syntax-parse-recurse ()
  (let ((*depth* (+ *depth* 1))
        (parse-tree ()))
    (do
      ((sym (table-pop *symbol-table*)
            (table-pop *symbol-table*)))
      ((null sym))
      (let ((token-type (car sym)))
        (cond ((equal token-type 'rightp-dt)   ;; Handle closing parenthesis specially
               (return))
              ((equal token-type 'leftp-dt)   ;; Recurse on opening parenthesis
               (let ((subtree (syntax-parse-recurse)))
                 (setf parse-tree
                       (cons subtree parse-tree))))
              (T (setf parse-tree
                       (cons sym parse-tree))
                ))))
    (nreverse parse-tree)))
