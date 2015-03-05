(load "tokens")

(defparameter gN (list 'S
                       'exprs 'exprlist
                       'constants
                       'ids 'idlist
                       'opers 'binops 'unops
                       'prims
                       'stmts))   ; Set of non-terminals
(defparameter gT `(list ,@constants
                        ,@delimiters
                        ,@identifiers
                        ,@operators
                        ,@primitives
                        ,@statements))   ; Set of terminals
(defparameter gG (list (list 'S          (list 'leftp-dt 'rightp-dt))
                       (list 'S          (list 'leftp-dt 'S 'rightp-dt))
                       (list 'S          (list 'S 'S))
                       (list 'S          (list 'exprs))
                       (list 'exprs      (list 'opers))
                       (list 'exprs      (list 'stmts))
                       (list 'exprlist   (list 'exprs))
                       (list 'exprlist   (list 'exprs 'exprlist))

                       (list 'constants  (list 'boolean-ct))
                       (list 'constants  (list 'integer-ct))
                       (list 'constants  (list 'real-ct))
                       (list 'constants  (list 'string-ct))

                       (list 'ids        (list 'function-it))
                       (list 'ids        (list 'variable-it))
                       (list 'idlist     (list 'leftp-dt 'ids 'prims 'rightp-dt))
                       (list 'idlist     (list 'leftp-dt 'ids 'prims 'rightp-dt 'idlist))

                       (list 'opers      (list 'leftp-dt 'assign-st 'ids 'opers 'rightp-dt))
                       (list 'opers      (list 'leftp-dt 'binops 'opers 'opers 'rightp-dt))
                       (list 'opers      (list 'leftp-dt 'unops 'opers 'rightp-dt))
                       (list 'opers      (list 'constants))
                       (list 'opers      (list 'ids))
                       (list 'binops     (list 'binop-ot))
                       (list 'unops      (list 'unop-ot))

                       (list 'prims      (list 'boolean-pt))
                       (list 'prims      (list 'integer-pt))
                       (list 'prims      (list 'real-pt))
                       (list 'prims      (list 'string-pt))

                       (list 'stmts      (list 'leftp-dt 'stdout-st 'opers 'rightp-dt))
                       (list 'stmts      (list 'leftp-dt 'if-st 'exprs 'exprs 'exprs 'rightp-dt))
                       (list 'stmts      (list 'leftp-dt 'if-st 'exprs 'exprs 'rightp-dt))
                       (list 'stmts      (list 'leftp-dt 'while-st 'exprs 'exprlist 'rightp-dt))
                       (list 'stmts      (list 'leftp-dt 'let-st 'leftp-dt 'idlist 'rightp-dt 'rightp-dt))
                       ))   ; Set of productions
(defparameter gS 'S)   ; Start symbol

;;; Define global grammar
(defparameter *grammar* (list gN gT gG gS))
