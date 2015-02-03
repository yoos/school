#!/usr/bin/sbcl --script

(load "tokens")
(load "states")
;(use-package #:lexer-tokens)

;;; Buffer in which to store token as we build it up
(defparameter *token* (make-array 0
                                  :element-type 'character
                                  :fill-pointer 0
                                  :adjustable t))
(defparameter *state* :find-token)    ; FSA state
(defparameter *type* :unknown-t)      ; Current token type

(defun letter? (c)
  (or (and (string>= c "A") (string<= c "Z"))
      (and (string>= c "a") (string<= c "z"))))

(defun number? (c)
  (or (and (string>= c "0") (string<= c "9"))
      (char= c #\.)
      (char= c #\e)))

(defun digit(c)   ; Combine with number? somehow
  (and (string>= c "0") (string<= c "9")))

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
  (format T "[TOKEN] <~A ~A>~%" token-type token-string)
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
       ;; More letters, non-numerals, etc?
       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((not (letter? c))   ; We should allow numbers, too
          (unread-char c istream)
          (defparameter *state* :store-token))
         (vector-push-extend c *token*))
       (cond
         ;; Check if primitive type
         ((string= *token* "bool")
          (defparameter *type* :boolean-tt))
         ((string= *token* "int")
          (defparameter *type* :integer-tt))
         ((string= *token* "real")
          (defparameter *type* :real-tt))
         ((string= *token* "string")
          (defparameter *type* :string-tt))

         ;; Check if statement
         ((string= *token* "stdout")
          (defparameter *type* :stdout-st))
         ((string= *token* "if")
          (defparameter *type* :if-st))
         ((string= *token* "while")
          (defparameter *type* :while-st))
         ((string= *token* "let")
          (defparameter *type* :let-st))

         ;; Fallback
         (T
           (defparameter *type* :identifier-tt))
         )
       )

      ;; String
      ((char= c #\")
       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((char= c #\")   ; Read until next quotation mark
          (vector-push-extend c *token*)   ; Push one more time. TODO(yoos): avoid this
          (defparameter *state* :store-token))
         (vector-push-extend c *token*))

       (defparameter *type* :string-ct))

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
            (defparameter *type* :real-tt)))
         )
       )

      ;; Op
      ((member c '(#\( #\)
                   #\+ #\- #\* #\/ #\^ #\%
                   #\= #\> #\< #\! #\: #\;
                   ))   ; The semicolon above screws up syntax highlighting.
       (defparameter *type* :op-t)
       (defparameter *state* :store-token)
       )

      ;; Whitespace
      ((or (string= c " ")
           (string= c #\tab))
       (clear-token)
       )

      ;; Newline
      ((string= c #\linefeed)
       (clear-token))

      ;; Fallback
      (T
        (format T "[Token FSA] Unknown: ~A~%" c))
      )

    ;; Store token and reset FSA
    (if (equal *state* :store-token)
      (store-token *type* *token*))
    )
  )


