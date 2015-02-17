(load "tokens")

(defparameter gN '(:constant-t
                    :identifier-t
                    :primitive-t
                    :statement-t))   ; Set of non-terminals
(defparameter gT `(,@constants
                    ,@identifiers
                    ,@primitives
                    ,@statements
                    :op-t))   ; Set of terminals
(defparameter gG '())   ; Set of productions
(defparameter gS :start-symbol)   ; Start symbol

;;; Define global grammar
(defparameter *grammar* (list gN gT gG gS))
