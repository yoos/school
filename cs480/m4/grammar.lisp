;;;; *GRAMMAR* definition
;;;
;;; S     -> EPSILON
;;;        | Sexpr S
;;;        | ( PS ) S
;;; Sexpr -> Soper
;;; PS    -> EPSILON
;;;        | Pexpr
;;;        | S
;;; Soper -> const | id
;;; Pexpr -> Poper | Pstmt
;;; Poper -> := id oper
;;;        | binop oper oper
;;;        | unop oper
;;; Pstmt -> if expr expr else
;;;        | while expr exprs
;;;        | let ( ids )
;;;        | stdout oper
;;; oper  -> Soper | ( Poper )
;;; expr  -> Sexpr | ( Pexpr )
;;; else  -> EPSILON | expr
;;; ids   -> EPSILON | ( id prim ) ids

(load "tokens")

; TODO: try to define the grammar as an S-expression
;(let ((S     (list (list exprs)))
;      (exprs (list ()
;                   (list Sexpr exprs)
;                   (list 'leftp-dt Pexpr 'rightp-dt exprs)))
;      (Sexpr (list (list Soper)))
;      (Pexpr (list ()
;                   (list exprs)
;                   (list Poper)
;                   (list Pstmt)))
;      (Soper (list (list const)
;                   (list id)))
;      (Poper (list (list 'assign-st id oper)
;                   (list binop oper oper)
;                   (list unop oper)))
;      (Pstmt (list (list 'if-st expr expr else)
;                   (list 'while-st expr exprs)
;                   (list 'let-st 'leftp-dt ids 'rightp-dt)
;                   (list 'stdout-st oper)))
;      (oper  (list (list Soper)
;                   (list 'leftp-dt Poper 'rightp-dt)))
;      (expr  (list (list Sexpr)
;                   (list 'leftp-dt Pexpr 'rightp-dt)))
;      (else  (list ()
;                   (list expr)))
;      (ids   (list ()
;                   (list 'leftp-dt id prim 'rightp-dt ids)))
;      (const constants)
;      (id    identifiers)
;      (prim  primitives)))

;;; TODO: Update this, though it's not currently being used.
(let* (;; Set of productions
       (gP (list (cons 'S        (list '+S))
                 (cons 'S        (list '_S))
                 (cons '+S       (list '_S 'S))
                 (cons '_S       (list 'nil))
                 (cons '_S       (list 'leftp-dt 'S 'rightp-dt))
                 (cons '_S       (list 'expr))
                 (cons 'nil      (list 'leftp-dt 'rightp-dt))

                 (cons 'expr     (list 'oper))
                 (cons 'expr     (list 'stmt))

                 (cons 'oper     (list 'leftp-dt 'Poper 'rightp-dt))
                 (cons 'oper     (list 'const))
                 (cons 'oper     (list 'id))
                 (cons 'Poper    (list 'assign-st 'id 'oper))
                 (cons 'Poper    (list 'binop-ot 'oper 'oper))
                 (cons 'Poper    (list 'unop-ot 'oper))

                 (cons 'stmt     (list 'leftp-dt 'Pstmt 'rightp-dt))
                 (cons 'Pstmt    (list 'ifstmt))
                 (cons 'Pstmt    (list 'elifstmt))
                 (cons 'Pstmt    (list 'while-st 'expr 'exprs))
                 (cons 'Pstmt    (list 'let-st 'leftp-dt 'ids 'rightp-dt))
                 (cons 'Pstmt    (list 'stdout-st 'oper))
                 (cons 'ifstmt   (list 'if-st 'expr 'expr))
                 (cons 'elifstmt (list 'if-st 'expr 'expr 'expr))

                 (cons 'exprs    (list '+exprs))
                 (cons 'exprs    (list 'expr))
                 (cons '+exprs   (list 'expr 'exprs))

                 (cons 'ids      (list '+ids))
                 (cons 'ids      (list 'leftp-dt 'id 'prim 'rightp-dt))
                 (cons '+ids     (list 'leftp-dt 'id 'prim 'rightp-dt 'ids))

                 ;; Singular constant categories
                 (list 'const (list 'boolean-ct))
                 (list 'const (list 'integer-ct))
                 (list 'const (list 'real-ct))
                 (list 'const (list 'string-ct))
                 (list 'id    (list 'function-it))
                 (list 'id    (list 'variable-it))
                 (list 'prim  (list 'boolean-pt))
                 (list 'prim  (list 'integer-pt))
                 (list 'prim  (list 'real-pt))
                 (list 'prim  (list 'string-pt))

                 ;; Former non-left-factored grammar
                 ; (list 'S     (list 'leftp-dt    'rightp-dt 'S))
                 ; (list 'S     (list 'leftp-dt 'S 'rightp-dt 'S))
                 ; (list 'S     (list 'expr                   'S))
                 ; (list 'S     (list 'leftp-dt    'rightp-dt))
                 ; (list 'S     (list 'leftp-dt 'S 'rightp-dt))
                 ; (list 'S     (list 'expr))

                 ; (list 'expr  (list 'oper))
                 ; (list 'expr  (list 'stmt))

                 ; (list 'oper  (list 'leftp-dt 'assign-st 'id   'oper 'rightp-dt))
                 ; (list 'oper  (list 'leftp-dt 'binop-ot  'oper 'oper 'rightp-dt))
                 ; (list 'oper  (list 'leftp-dt 'unop-ot   'oper       'rightp-dt))
                 ; (list 'oper  (list 'const))
                 ; (list 'oper  (list 'id))

                 ; (list 'stmt  (list 'leftp-dt 'if-st     'expr     'expr  'expr      'rightp-dt))
                 ; (list 'stmt  (list 'leftp-dt 'if-st     'expr     'expr             'rightp-dt))
                 ; (list 'stmt  (list 'leftp-dt 'while-st  'expr     'exprs            'rightp-dt))
                 ; (list 'stmt  (list 'leftp-dt 'let-st    'leftp-dt 'ids   'rightp-dt 'rightp-dt))
                 ; (list 'stmt  (list 'leftp-dt 'stdout-st 'oper                       'rightp-dt))

                 ; (list 'exprs (list 'expr 'exprs))
                 ; (list 'exprs (list 'expr))

                 ; (list 'ids   (list 'leftp-dt 'id 'prim 'rightp-dt 'ids))
                 ; (list 'ids   (list 'leftp-dt 'id 'prim 'rightp-dt))
                 ))

       ;; Generate non-terminals from all LHS symbols in gG
       ;; TODO: This is a bit ugly, as we get duplicates...
       (gN (mapcar (lambda (R) (car R)) gP))

       ;; Set of terminals
       (gT `(list ,@constants
                  ,@delimiters
                  ,@identifiers
                  ,@operators
                  ,@primitives
                  ,@statements))

       ;; Start symbol
       (gS 'S))

  ;; Define global grammar
  (defparameter *grammar* (list gN gT gP gS)))
