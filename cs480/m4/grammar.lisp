;;;; *GRAMMAR* definition
;;;
;;; The following is the IBTL grammar in BNF:
;;;
;;;   S     -> ( ) S   |
;;;            ( S ) S |
;;;            expr S  |
;;;            ( )     |
;;;            ( S )   |
;;;            expr
;;;   expr  -> oper |
;;;            stmt
;;;   oper  -> ( := id oper )      |
;;;            ( binop oper oper ) |
;;;            ( unop oper )       |
;;;            const               |
;;;            id
;;;   stmt  -> ( if expr expr expr ) |
;;;            ( if expr expr )      |
;;;            ( while expr exprs )  |
;;;            ( let ( ids ) )       |
;;;            ( stdout oper )
;;;   exprs -> expr exprs |
;;;            expr
;;;   ids   -> ( id prim ) ids |
;;;            ( id prim )

(load "tokens")

(defparameter gN (list 'S 'expr 'oper 'stmt 'exprs 'ids))   ; Set of non-terminals

(defparameter gT `(list ,@constants
                        ,@delimiters
                        ,@identifiers
                        ,@operators
                        ,@primitives
                        ,@statements))   ; Set of terminals

(defparameter gG (list (list 'S     (list 'leftp-dt    'rightp-dt 'S))
                       (list 'S     (list 'leftp-dt 'S 'rightp-dt 'S))
                       (list 'S     (list 'expr                   'S))
                       (list 'S     (list 'leftp-dt    'rightp-dt))
                       (list 'S     (list 'leftp-dt 'S 'rightp-dt))
                       (list 'S     (list 'expr))

                       (list 'expr  (list 'oper))
                       (list 'expr  (list 'stmt))

                       (list 'oper  (list 'leftp-dt 'assign-st 'id   'oper 'rightp-dt))
                       (list 'oper  (list 'leftp-dt 'binop-ot  'oper 'oper 'rightp-dt))
                       (list 'oper  (list 'leftp-dt 'unop-ot   'oper       'rightp-dt))
                       (list 'oper  (list 'const))
                       (list 'oper  (list 'id))

                       (list 'stmt  (list 'leftp-dt 'if-st     'expr     'expr  'expr      'rightp-dt))
                       (list 'stmt  (list 'leftp-dt 'if-st     'expr     'expr             'rightp-dt))
                       (list 'stmt  (list 'leftp-dt 'while-st  'expr     'exprs            'rightp-dt))
                       (list 'stmt  (list 'leftp-dt 'let-st    'leftp-dt 'ids   'rightp-dt 'rightp-dt))
                       (list 'stmt  (list 'leftp-dt 'stdout-st 'oper                       'rightp-dt))

                       (list 'exprs (list 'expr 'exprs))
                       (list 'exprs (list 'expr))

                       (list 'ids   (list 'leftp-dt 'id 'prim 'rightp-dt 'ids))
                       (list 'ids   (list 'leftp-dt 'id 'prim 'rightp-dt))

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
                       ))   ; Set of productions

(defparameter gS 'S)   ; Start symbol

;;; Define global grammar
(defparameter *grammar* (list gN gT gG gS))
