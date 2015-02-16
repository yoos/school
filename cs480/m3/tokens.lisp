(defpackage #:lexer-tokens)

;;; Constants and identifiers
;;;
;;; These will be stored as (type, token) pairs in the symbol table, where
;;; token is a string.
(deftype constant-t ()     ; Constants
  '(member :boolean-ct
           :integer-ct
           :real-ct
           :string-ct))
(deftype identifier-t ()   ; Identifiers
  '(member :function-it
           :variable-it))

;;; Types and statements
;;;
;;; Stored as (type, NIL)
(deftype primitive-t ()         ; Primitive types
  '(member :boolean-pt
           :integer-pt
           :real-pt
           :string-pt))
(deftype statement-t ()    ; Statements
  '(member :stdout-st
           :if-st
           :while-st
           :let-st
           :assign-st))

;;; Encapsulate the above types as a token type
(deftype token-t ()
  '(member constant-t
           identifier-t
           op-t
           primitive-t
           statement-t
           unknown-t))

