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

(defun digit? (c)
  (and (string>= c "0") (string<= c "9")))

(defun buildtoken (istream token)
  (let (c '(read-char istream nil))
    (vector-push-extend c token)
    c))

;(defun update-state (fsa-state fsa-type)
;  (defvar *state* fsa-state)
;  (defvar *type* fsa-type))
;
(defun store-token (token-type token-string)
  (format T "[TOKEN] <~A ~A>~%" token-type token-string)
  (defparameter *state* :find-token)
  (defparameter *type* :unknown-t)
  (defparameter *token* (make-array 0
                                    :element-type 'character
                                    :fill-pointer 0
                                    :adjustable t)))


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
          (unread-char c istream))
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

       ;; Store token and reset FSA
       (store-token *type* *token*)
       )

      ;; 0-9
      ((digit? c)
       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((not (digit? c))   ; We should allow exponents and at most one decimal point
          (unread-char c istream))
         (vector-push-extend c *token*))
       (cond
         ((or (string= *token* "e")
              (string= *token* "."))
          (defparameter *type* :real-tt))
         )
       (store-token *type* *token*)
       ;(format T "[Token FSA] Number~%")
       )

      ;; String
      ((string= c #\")
       (format T "[Token FSA] String~%"))

      ;; Op
      ((member c '(#\( #\)
                   #\+ #\- #\* #\/ #\^ #\%
                   #\= #\> #\< #\! #\: #\;
                   ))   ; The semicolon above screws up syntax highlighting.
       (format T "[Token FSA] Op: ~A~%" c))

      ;; Whitespace
      ((or (string= c " ")
           (string= c #\tab))
       ;(format T "[Token FSA] Whitespace~%")
       )

      ;; Newline
      ((string= c #\linefeed)
       (format T "[Token FSA] Newline~%"))

      ;; Fallback
      (T
        (format T "[Token FSA] Unknown: ~A~%" c))
      ))
  )


