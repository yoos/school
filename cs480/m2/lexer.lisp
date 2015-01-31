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


(with-open-file (stream "inputs/1.txt")
  (cond
    ;; Find new token
    ((eq *state* :find-token)
     (format T "[State FSA] Find token")

     ;; Read in one character
     (do
       ((c (read-char stream nil)
           (read-char stream nil)))
       ((null c))
       (vector-push-extend c *token*)   ; Append char to token

       ;; Match
       (cond
         ;; A-Z, a-z
         ((or (and (string>= c "A") (string<= c "Z"))
              (and (string>= c "a") (string<= c "z")))
          (format T "[Token FSA] Character~%"))

         ;; 0-9
         ((and (string>= c "0") (string<= c "9"))
          (format T "[Token FSA] Number~%"))

         ;; String
         ((string= c #\")
          (format T "[Token FSA] String~%"))

         ;; Op
         ((member c '(#\( #\)
                      #\+ #\- #\* #\/ #\^ #\%
                      #\= #\> #\< #\! #\: #\;))
          (format T "[Token FSA] Op: ~A~%" c))


         ;; Whitespace
         ((or (string= c " ")
              (string= c #\tab))
          (format T "[Token FSA] Whitespace~%"))

         ;; Newline
         ((string= c #\linefeed)
          (format T "[Token FSA] Newline~%"))

         ;; Fallback
         (T
           (format T "[Token FSA] Unknown: ~A~%" c))
         )
       ))

    ;; Token found, so store in table
    ((eq *state* :store-token)
     (format T "[State FSA] Store token~%"))

    ;; Fallback case
    (T
      (format T "[FSA] Unknown state~%"))))

(format T *token*)

