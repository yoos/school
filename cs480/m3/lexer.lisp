#!/usr/bin/sbcl --script

(load "tokens")
(load "states")
;(use-package #:lexer-tokens)

;;; Buffer in which to store token as we build it up
(defparameter *token* (make-array 0
                                  :element-type 'character
                                  :fill-pointer 0
                                  :adjustable T))
(defparameter *state* :find-token)    ; FSA state
(defparameter *type* :unknown-t)      ; Current token type

(defparameter *token-list* (make-array 0
                                       :element-type 'list
                                       :fill-pointer 0
                                       :adjustable T))

(defun letter? (c)
  (or (and (string>= c "A") (string<= c "Z"))
      (and (string>= c "a") (string<= c "z"))))

(defun number? (c)
  (or (and (string>= c "0") (string<= c "9"))
      (char= c #\.)
      (char= c #\e)))

(defun digit? (c)   ; Combine with number? somehow
  (and (string>= c "0") (string<= c "9")))

; TODO: ! is not an op
; TODO: treat sin cos tan as unary ops
(defun op? (c)
  (member c '(#\( #\)
              #\+ #\- #\* #\/ #\^ #\%
              #\= #\> #\< #\! #\: #\;
              )))

(defun build-token (istream token)
  (let (c '(read-char istream nil))
    (vector-push-extend c token)
    c))

(defun clear-token ()
  (defparameter *token* (make-array 0
                                    :element-type 'character
                                    :fill-pointer 0
                                    :adjustable t)))

(defun store-token (token-type token-string)
  (format T "[STORE-TOKEN] <~A ~A>~%" token-type token-string)
  (vector-push-extend (list token-type token-string) *token-list*)
  (defparameter *state* :find-token)
  (defparameter *type* :unknown-t)
  (clear-token))


(with-open-file (istream "inputs/2.txt")
  ;; Read in one character
  (do
    ((c (read-char istream NIL)     ; Start with first char read
        (read-char istream NIL)))   ; Read another char each step
    ((null c))                      ; End when c is null
    (vector-push-extend c *token*)   ; Append char to token

    (cond
      ;; Letters
      ((letter? c)
       (defparameter *type* :identifier-t)

       ;; More letters, non-numerals, etc?
       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((and (not (letter? c))
               (not (digit? c)))   ; We should allow numbers, too
          (unread-char c istream)
          (defparameter *state* :store-token))
         (vector-push-extend c *token*))
       (cond
         ;; Primitive types
         ((string= *token* "bool")
          (defparameter *type* :boolean-pt))
         ((string= *token* "int")
          (defparameter *type* :integer-pt))
         ((string= *token* "real")
          (defparameter *type* :real-pt))
         ((string= *token* "string")
          (defparameter *type* :string-pt))

         ;; Statements
         ((string= *token* "stdout")
          (defparameter *type* :stdout-st))
         ((string= *token* "if")
          (defparameter *type* :if-st))
         ((string= *token* "while")
          (defparameter *type* :while-st))
         ((string= *token* "let")
          (defparameter *type* :let-st))

         ;; Boolean constants
         ((or (string= *token* "true")
              (string= *token* "false"))
          (defparameter *type* :boolean-ct))

         ;; Fallback
         (T ()))
       )

      ;; String
      ((char= c #\")
       (defparameter *type* :string-ct)

       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((char= c #\")   ; Read until next quotation mark
          (vector-push-extend c *token*)   ; Push one more time. TODO(yoos): avoid this
          (defparameter *state* :store-token))
         (vector-push-extend c *token*)))


      ;; Number
      ((number? c)
       (defparameter *type* :integer-ct)

       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((if (equal *type* :real-ct)
            (not (digit? c))
            (not (number? c)))
          (unread-char c istream)
          (defparameter *state* :store-token))
         (vector-push-extend c *token*)
         ;; Check if real. Doesn't have to be cond here since only one condition, but just in case..
         (cond
           ((or (char= c #\e)
                (char= c #\.))
            (defparameter *type* :real-pt))))
       )

      ;; Op (and assign)
      ((op? c)   ; The semicolon above screws up syntax highlighting.
       (defparameter *type* :op-t)

       ;; Try reading one more before proceeding to :store-token.
       (let ((c (read-char istream NIL)))
         (vector-push-extend c *token*)
         (cond
           ;; Multichar ops
           ((or (string= *token* ">=")
                (string= *token* "<=")
                (string= *token* "!=")))

           ;; Assign statement
           ((string= *token* ":=")
            (defparameter *type* :assign-st))

           ;; Unread
           (T
             (unread-char c istream)
             (vector-pop *token*))
             )
         (defparameter *state* :store-token)
         )
       )

      ;; Whitespace
      ((or (string= c " ")
           (string= c #\tab))
       (clear-token)
       )

      ;; Newline
      ((string= c #\linefeed)
       (format T "~%")   ; Print out newline for readability
       (clear-token))

      ;; Fallback
      (T
        (format T "[Token FSA] Unknown: ~A~%" c))
      )

    ;; Store token and reset FSA
    (if (equal *state* :store-token)
      (store-token *type* *token*))
    )

  ;; Print symbol table
  (format T "Symbol table:~%~S" *token-list*)
  )


