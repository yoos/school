(load "tokens")

(defparameter gN '(:S
                    :exprs :exprlist
                    :constants :bools :ints :reals :strings
                    :ids :idlist
                    :opers :binops :unops
                    :prims
                    :stmts :printstmts :ifstmts :whilestmts :letstmts))   ; Set of non-terminals
(defparameter gT `(,@constants
                    ,@delimiters
                    ,@identifiers
                    ,@operators
                    ,@primitives
                    ,@statements))   ; Set of terminals
(defparameter gG (list (list :S        (list '(:leftp-dt :rightp-dt)
                                             '(:leftp-dt :S :rightp-dt)
                                             '(:S :S)
                                             '(:exprs)))
                       (list :exprs    (list '(:opers)
                                             '(:stmts)))
                       (list :exprlist (list '(:exprs)
                                             '(:exprs :exprlist)))

                       (list :constants (list '(:bools)
                                              '(:ints)
                                              '(:reals)
                                              '(:strings)))
                       (list :bools     (list '(:boolean-ct)))
                       (list :ints      (list '(:integer-ct)))
                       (list :reals     (list '(:real-ct)))
                       (list :strings   (list '(:string-ct)))

                       (list :ids    (list '(:function-it)
                                           '(:variable-it)))
                       (list :idlist (list '(:leftp-dt :ids :prims :rightp-dt)
                                           '(:leftp-dt :ids :prims :rightp-dt :idlist)))

                       (list :opers  (list '(:leftp-dt :assign-st :ids :opers :rightp-dt)
                                           '(:leftp-dt :binops :opers :opers :rightp-dt)
                                           '(:leftp-dt :unops :opers :rightp-dt)
                                           '(:constants)
                                           '(:ids)))
                       (list :binops (list '(:binop-ot)))
                       (list :unops  (list '(:unop-ot)))

                       (list :prims (list '(:boolean-pt)
                                          '(:integer-pt)
                                          '(:real-pt)
                                          '(:string-pt)))

                       (list :stmts      (list '(:ifstmts)
                                               '(:whilestmts)
                                               '(:letstmts)
                                               '(:printstmts)))
                       (list :printstmts (list '(:leftp-dt :stdout-st :opers :rightp-dt)))
                       (list :ifstmts    (list '(:leftp-dt :if-st :exprs :exprs :exprs :rightp-dt)
                                               '(:leftp-dt :exprs :exprs)))
                       (list :whilestmts (list '(:leftp-dt :while-st :exprs :exprlist :rightp-dt)))
                       (list :letstmts   (list '(:leftp-dt :let-st :leftp-dt :idlist :rightp-dt :rightp-dt)))
                       ))   ; Set of productions
(defparameter gS :S)   ; Start symbol

;;; Define global grammar
(defparameter *grammar* (list gN gT gG gS))
