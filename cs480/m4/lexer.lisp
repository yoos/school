(load "tokens")
(load "states")

(defun letter? (c)
  (or (and (string>= c "A") (string<= c "Z"))
      (and (string>= c "a") (string<= c "z"))))

(defun number? (c)
  (or (and (string>= c "0") (string<= c "9"))
      (char= c #\-)   ; negative sign is first checked as number.
      (char= c #\.)
      (char= c #\e)))

(defun digit? (c)   ; Combine with number? somehow
  (and (string>= c "0") (string<= c "9")))

(defun op? (c)
  (member c (list #\( #\)
                  #\+ #\* #\/ #\^ #\%
                  #\= #\> #\< #\! #\:)))

(defun zero-array (eltype)
  (make-array 0
              :element-type eltype
              :fill-pointer 0
              :adjustable T))

(defparameter *token-list* (zero-array 'cons))
(defparameter *lexeme* (zero-array 'character))
(defparameter *state* 'find-token)
(defparameter *type* 'unknown-t)

(defun strmatch? (strlist)
  (if (null strlist)
    NIL
    (or (string= *lexeme* (car strlist)) (strmatch? (cdr strlist)))))

(defun store-token (token-type token-string)
                    (vector-push-extend (cons token-type token-string) *token-list*)
                    (setf *lexeme* (zero-array 'character)
                          *state* 'find-token
                          *type* 'unknown-t))
 
(defun lex (istream)
  (do
    ((c (read-char istream NIL)     ; Start with first char read
        (read-char istream NIL))    ; Read another char each step
     (*token-list* (zero-array 'cons))
     (*lexeme* (zero-array 'character))
     (*state* 'find-token)
     (*type* 'unknown-t))
    ((null c)                   ; End when c is null
     (vector-push-extend (cons 'eof T) *token-list*)
     (nreverse *token-list*))   ; Reverse symbol table so we can pop.
    (vector-push-extend c *lexeme*)   ; Append char to token

    (cond
      ;; Letters
      ((letter? c)
       ;; Consume
       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((and (not (letter? c))
               (not (digit? c)))   ; We should allow numbers, too
          (unread-char c istream)
          (setf *state* 'store-token))
         (vector-push-extend c *lexeme*))

       ;; Set type
       (setf *type*
             (cond
               ;; Constants
               ((strmatch? '("true" "false")) 'boolean-ct)

               ;; Operators
               ((strmatch? '("and" "or"))              'binop-t)
               ((strmatch? '("not" "sin" "cos" "tan")) 'unop-t)

               ;; Primitives
               ((strmatch? '("bool"))   'boolean-pt)
               ((strmatch? '("int"))    'integer-pt)
               ((strmatch? '("real"))   'real-pt)
               ((strmatch? '("string")) 'string-pt)

               ;; Statements
               ((strmatch? '("if"))     'if-st)
               ((strmatch? '("while"))  'while-st)
               ((strmatch? '("let"))    'let-st)
               ((strmatch? '("stdout")) 'stdout-st)

               ;; Fallback
               (T 'identifier-t))))

      ;; String
      ((char= c #\")
       (vector-pop *lexeme*)   ; Don't store the quotation mark

       ;; Set type
       (setf *type* 'string-ct)

       ;; Consume
       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((char= c #\")   ; Read until next quotation mark
          (setf *state* 'store-token))
         (vector-push-extend c *lexeme*)))

      ;; Number
      ((number? c)
       ;; Set type
       (setf *type* 'integer-ct)

       ;; Consume
       (do
         ((c (read-char istream NIL)
             (read-char istream NIL)))
         ((if (equal *type* 'real-ct)
            (not (digit? c))
            (not (number? c)))
          (unread-char c istream)
          (setf *state* 'store-token))
         (vector-push-extend c *lexeme*)
         ;; Update type if real.
         (cond
           ((or (char= c #\e)
                (char= c #\.))
            (setf *type* 'real-ct))))

       ;; Update type. A singular minus sign is a binop.
       (if (string= *lexeme* "-")
         (setf *type* 'binop-ot)))

      ;; Op (and assign)
      ((op? c)
       ;; Set type. Peek one char further for multichar ops.
       (let ((c (read-char istream NIL)))
         (vector-push-extend c *lexeme*)
         (setf *type*
               (cond
                 ;; Multichar ops
                 ((strmatch? '(">=" "<=" "!=")) 'binop-ot)

                 ;; Assign statement
                 ((strmatch? '(":=")) 'assign-st)

                 ;; Unread and process singlechar ops
                 (T (unread-char c istream)
                    (vector-pop *lexeme*)
                    (cond
                      ((strmatch? '("(")) 'leftp-dt)
                      ((strmatch? '(")")) 'rightp-dt)
                      ((strmatch? '("!")) 'unknown-t)
                      (T 'binop-ot))))))
       (setf *state* 'store-token))

      ;; Whitespace
      ((or (string= c " ")
           (string= c #\tab))
       (setf *lexeme* (zero-array 'character))
       )

      ;; Newline
      ((string= c #\linefeed)
       ;(format T "~%")   ; Print out newline for readability
       (setf *lexeme* (zero-array 'character)))

      ;; Fallback
      (T
        (setf *type* 'unknown-t)
        (setf *state* 'store-token))
      )

    ;; Store token and reset FSA
    (if (equal *state* 'store-token)
      (store-token *type* *lexeme*))
    )
  )
