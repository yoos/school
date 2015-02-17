(load "tokens")
(load "states")

;;; Buffer in which to store token as we build it up
(defparameter *lexeme* (make-array 0
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

(defun op? (c)
  (member c (list #\( #\)
                  #\+ #\- #\* #\/ #\^ #\%
                  #\= #\> #\< #\! #\: #\;)))

;(defun build-token (istream token)
;  (let (c '(read-char istream nil))
;    (vector-push-extend c token)
;    c))

(defun clear-token ()
  (defparameter *lexeme* (make-array 0
                                     :element-type 'character
                                     :fill-pointer 0
                                     :adjustable t)))

(defun store-token (token-type token-string)
  (format T "[LEX] (~A ~A)~%" token-type token-string)
  (vector-push-extend (list token-type token-string) *token-list*)
  (defparameter *state* :find-token)
  (defparameter *type* :unknown-t)
  (clear-token))


(defun lex (istream)
  ;; Read in one character
  (do
    ((c (read-char istream NIL)     ; Start with first char read
        (read-char istream NIL)))   ; Read another char each step
    ((null c))                      ; End when c is null
    (vector-push-extend c *lexeme*)   ; Append char to token

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
         (vector-push-extend c *lexeme*))
       (cond
         ;; Constants
         ((or (string= *lexeme* "true")
              (string= *lexeme* "false"))
          (defparameter *type* :boolean-ct))

         ;; Unary ops
         ((or (string= *lexeme* "not")
              (string= *lexeme* "sin")
              (string= *lexeme* "cos")
              (string= *lexeme* "tan"))
          (defparameter *type* :unop-t))

         ;; Binary ops
         ((or (string= *lexeme* "and")
              (string= *lexeme* "or"))
          (defparameter *type* :binop-t))

         ;; Primitives
         ((string= *lexeme* "bool")
          (defparameter *type* :boolean-pt))
         ((string= *lexeme* "int")
          (defparameter *type* :integer-pt))
         ((string= *lexeme* "real")
          (defparameter *type* :real-pt))
         ((string= *lexeme* "string")
          (defparameter *type* :string-pt))

         ;; Statements
         ((string= *lexeme* "stdout")
          (defparameter *type* :stdout-st))
         ((string= *lexeme* "if")
          (defparameter *type* :if-st))
         ((string= *lexeme* "while")
          (defparameter *type* :while-st))
         ((string= *lexeme* "let")
          (defparameter *type* :let-st))

         ;; Fallback
         (T ()))
       )

      ;; String
      ((char= c #\")
       (defparameter *type* :string-ct)
       (vector-pop *lexeme*)   ; Don't store the quotation mark

       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((char= c #\")   ; Read until next quotation mark
          (defparameter *state* :store-token))
         (vector-push-extend c *lexeme*)))


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
         (vector-push-extend c *lexeme*)
         ;; Check if real. Doesn't have to be cond here since only one condition, but just in case..
         (cond
           ((or (char= c #\e)
                (char= c #\.))
            (defparameter *type* :real-pt))))
       )

      ;; Op (and assign)
      ((op? c)
       (defparameter *type* :binop-t)

       ;; Try reading one more before proceeding to :store-token.
       (let ((c (read-char istream NIL)))
         (vector-push-extend c *lexeme*)
         (cond
           ;; Multichar ops
           ((or (string= *lexeme* ">=")
                (string= *lexeme* "<=")
                (string= *lexeme* "!=")))

           ;; Assign statement
           ((string= *lexeme* ":=")
            (defparameter *type* :assign-st))

           ;; Unread
           (T
             (unread-char c istream)
             (vector-pop *lexeme*))
           )

         ;; Categorize "!" as unknown lexeme
         (cond
           ((string= *lexeme* "(")
            (defparameter *type* :leftp-dt))
           ((string= *lexeme* ")")
            (defparameter *type* :rightp-dt))
           ((string= *lexeme* ";")
            (defparameter *type* :semicolon-dt))
           ((string= *lexeme* "!")
            (defparameter *type* :unknown-t)))

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
        (defparameter *type* :unknown-t)
        (defparameter *state* :store-token))
      )

    ;; Store token and reset FSA
    (if (equal *state* :store-token)
      (store-token *type* *lexeme*))
    )

  ;; Print symbol table
  ;(format T "Symbol table:~%~S~%" *token-list*)

  (nreverse *token-list*)   ; Reverse symbol table so we can pop.
  )
