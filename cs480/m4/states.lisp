(defpackage #:lexer-states)

;;; The current states is a (state, type) pair
(deftype state-t ()
  '(member :find-token
           :store-token))

;;; Define final states
(deftype final-state ()
  '(member (cons :find-token constant-t)
           ))


