(load "tokens")

(defparameter *sym* ())
(defparameter *symbol-table* ())
(defparameter *depth* 0)
(defparameter *accept* T)
(defparameter *idx* 0)

(defun init-globals (symbol-table)
  (setf *sym* ()
        *symbol-table* symbol-table
        *depth* 0
        *accept* T
        *idx* 0
        ))

;;; Convenience function so popping from symbol table fails gracefully.
(defun table-pop (symbol-table)
  ;(if (> (fill-pointer symbol-table) 0)
  ;  (vector-pop symbol-table)))
  (cond ((< *idx* (length symbol-table))
         (let ((ret (nth *idx* symbol-table)))
           (setf *idx* (+ *idx* 1))
           ret)
         )
        (T
          (cons 'error NIL))))

(defun get-sym ()
  (setf *sym* (table-pop *symbol-table*)))

(defun unget-sym (n)
  (format *enable-debug* "UNGETTING SYMBOL ~S~%" *sym*)
  ;(vector-push-extend *sym* *symbol-table*)
  (setf *idx* (- *idx* n))
  (defparameter *sym* (get-sym))
  NIL)

(defun try (&rest undos)
  (let ((accept T)
        (undo-count 0))
    (loop for u in undos do
          (cond ((>= u 0)
                 (setf undo-count (+ undo-count u)))
    (cond ((accept)
           (unget-sym undo-count)
           NIL)
          (T T))))

    ; TODO: This needs to return number of undos as well as accept/reject decision!

(defun parse-reject (str)
  (format *enable-debug* "~,,v,@A" (* *depth* 2) (format NIL "[REJECT ~S] ~@?" *depth* str))
  (setf *accept* NIL)
  ;(unget-sym)
  ;(format *enable-debug* "~,,v,@A" (* *depth* 2) (format NIL "[REJECT ~S] *sym* reverted to ~S~%" *depth* *sym*))
  NIL)

(defun parse-info (str &optional (enable-indent T))
  (if enable-indent
    (format *enable-debug* "~,,v,@A" (* 2 *depth*) (format NIL "[INFO ~S] ~@?" *depth* str))
    (format *enable-debug* "~@?" str))
  T)

;;; Like accept, but with backtracking
(defun parse-term (token-type)
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Symbol to accept is ~S.. " *sym*))

    ;; First get a symbol
    (get-sym)

    ;; Check against given type. If it matches, return 1 as we've consumed
    ;; 1 symbol. Otherwise, return -1. Similarly, non-term parsers will return
    ;; the number of symbols consumed, otherwise -1.
    (cond ((equal (car *sym*) token-type)
           (parse-info (format NIL "accepting ~S. Next is ~S~%" token-type (get-sym)) NIL)
           1)
          (T
            (unget-sym)
            (parse-info (format NIL "rejecting ~S~%" token-type) NIL)
            -1))))

(defun accept (token-type)
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Symbol to accept is ~S.. " *sym*))
    (cond ((equal (car *sym*) token-type)
           (parse-info (format NIL "accepting ~S. Next is ~S~%" token-type (get-sym)) NIL)
           T)
          (T
            (parse-info (format NIL "rejecting ~S~%" token-type) NIL)
            NIL)))
  )

(defun expect (token-type)
  (cond ((accept token-type) T)
        (T
          (parse-reject (format NIL "Expected ~S, got ~S instead~%" token-type (car *sym*))) NIL)))

(defun parse (symbol-table)
  (init-globals symbol-table)
  (get-sym)
  (parse-info (format NIL "Parsing on ~S~%" *sym*))
  (and (parse-S)
       (expect 'eof)
       *accept*)
  )

(defun parse-S ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing S on ~S~%" *sym*))
    (cond ((and (parse-info "Trying S -> ( S ) S..~%")
                (accept 'leftp-dt)
                (parse-S)
                (expect 'rightp-dt)
                (parse-S))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying S -> expr S..~%")
                (parse-expr)
                (parse-S))
           (parse-info "accepted~%") T)
          (T
            (parse-info "Parsed: S -> EPSILON~%") T
            ;(parse-info "Unable to parse S~%") NIL
            ))))

(defun parse-expr ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing expr on ~S~%" *sym*))
    (cond ((and (parse-info "Trying expr -> oper..~%")
                (parse-oper))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying expr -> stmt..~%")
                (parse-stmt))
           (parse-info "accepted~%") T)
          (T (parse-info "Parse failed on expr~%") NIL))))

(defun parse-oper ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing oper on ~S~%" *sym*))
    (cond ((and (parse-info "Trying oper -> ( Poper )..~%")
                (accept 'leftp-dt)
                (parse-Poper)
                (expect 'rightp-dt))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying oper -> const..~%")
                (parse-const))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying oper -> id..~%")
                (parse-id))
           (parse-info "accepted~%") T)
          (T (parse-info "Parse failed on oper~%") NIL))))

(defun parse-stmt ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing stmt on ~S~%" *sym*))
    (cond ((and (parse-info "Trying stmt -> ( Pstmt )..~%")
                (accept 'leftp-dt)
                (parse-Pstmt)
                (expect 'rightp-dt))
           (parse-info "accepted~%") T)
          (T (parse-info "Parse failed on stmt~%") NIL))))

(defun parse-Poper ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing Poper on ~S~%" *sym*))
    (cond ((and (parse-info "Trying Poper -> := id oper..~%")
                (accept 'assign-st)
                (parse-id)
                (parse-oper))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Poper -> binop oper oper..~%")
                (accept 'binop-ot)
                (parse-oper)
                (parse-oper))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Poper -> unop oper..~%")
                (accept 'unop-ot)
                (parse-oper))
           (parse-info "accepted~%") T)
          (T (parse-info "Parse failed on Poper~%") NIL))))

(defun parse-Pstmt ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing Pstmt on ~S~%" *sym*))
    (cond ((and (parse-info "Trying Pstmt -> if expr expr else..~%")
                (accept 'if-st)
                (parse-expr)
                (parse-expr)
                (parse-else))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Pstmt -> while expr exprs..~%")
                (accept 'while-st)
                (parse-expr)
                (parse-exprs))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Pstmt -> let ( ids )..~%")
                (accept 'let-st)
                (expect 'leftp-dt)
                (parse-ids)
                (expect 'rightp-dt))
           (parse-info "accepted~%") T)
          ((and (parse-info "Trying Pstmt -> stdout oper..~%")
                (accept 'stdout-st)
                (parse-oper))
           (parse-info "accepted~%") T)
          (T (parse-info "Parse failed on Pstmt~%") NIL))))

(defun parse-else ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing else on ~S~%" *sym*))
    (cond ((and (parse-info "Trying else -> expr..~%")
                (parse-expr))
           (parse-info "accepted~%") T)
          (T (parse-info "Parsed: else -> EPSILON~%") T))))

(defun parse-exprs ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing exprs on ~S~%" *sym*))
    (cond ((and (parse-info "Trying exprs -> expr exprs..~%")
                (parse-expr)
                (parse-exprs))
           (parse-info "accepted~%") T)
          (T (parse-info "Parsed: exprs -> EPSILON~%") T))))

(defun parse-ids ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing ids on ~S~%" *sym*))
    (cond ((and (parse-info "Trying ids -> ( id prim ) ids..~%")
                (accept 'leftp-dt)
                (parse-id)
                (parse-prim)
                (expect 'rightp-dt)
                (parse-ids))
           (parse-info "accepted~%") T)
          (T (parse-info "Parsed: ids -> EPSILON~%") T))))

(defun parse-const ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing const on ~S~%" *sym*))
    (cond ((accept 'boolean-ct)
           (parse-info "Parsed: const -> boolean-ct~%") T)
          ((accept 'integer-ct)
           (parse-info "Parsed: const -> integer-ct~%") T)
          ((accept 'real-ct)
           (parse-info "Parsed: const -> real-ct~%") T)
          ((accept 'string-ct)
           (parse-info "Parsed: const -> string-ct~%") T)
          (T (parse-info "Unable to parse const~%") NIL))))

(defun parse-id ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing id on ~S~%" *sym*))
    (cond ((accept 'function-it)
           (parse-info "Parsed: id -> function-ct~%") T)
          ((accept 'variable-it)
           (parse-info "Parsed: id -> variable-ct~%") T)
          (T (parse-info "Unable to parse id~%") NIL))))

(defun parse-prim ()
  (let ((*depth* (+ *depth* 1)))
    (parse-info (format NIL "Parsing prim on ~S~%" *sym*))
    (cond ((accept 'boolean-pt)
           (parse-info "Parsed: prim -> boolean-pt~%") T)
          ((accept 'integer-pt)
           (parse-info "Parsed: prim -> integer-pt~%") T)
          ((accept 'real-pt)
           (parse-info "Parsed: prim -> real-pt~%") T)
          ((accept 'string-pt)
           (parse-info "Parsed: prim -> string-pt~%") T)
          (T (parse-info "Unable to parse prim~%") NIL))))




(defun syntax-parse (symbol-table grammar depth)
  (let ((parse-tree (list (cons 'leftp-dt "("))))
    (do
      ((sym (table-pop symbol-table)
            (table-pop symbol-table)))
      ((null sym))
      (let ((token-type (car sym))
            (token      (cdr sym)))
        (cond ((equal token-type 'rightp-dt)   ;; Handle closing parenthesis specially
               (format T "~,,v,@A~%" (* 2 (- depth 1)) token)
               (return))
              ((equal token-type 'leftp-dt)   ;; Recurse on opening parenthesis
               (format T "~,,v,@A~%" (* 2 depth) token)
               (let ((subtree (syntax-parse symbol-table grammar (+ 1 depth))))
                 (setf parse-tree
                       (cons subtree parse-tree))))
              (T
                ;; Print to screen at correct indentation
                ;; See http://stackoverflow.com/questions/20072959/lisp-format-a-character-a-number-of-times
                (format T "~,,v,@A~%" (* 2 depth) token)
                (setf parse-tree
                      (cons sym parse-tree))
                ))))
    (setf parse-tree
          (cons (cons 'rightp-dt ")") parse-tree))
    (nreverse parse-tree)))
