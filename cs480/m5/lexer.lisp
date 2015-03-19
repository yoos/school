(load "tokens")
(load "states")
(load "util")

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

(defparameter *token-list* ())
(defparameter *lexeme* (zero-array 'character))
(defparameter *state* 'find-token)
(defparameter *type* 'unknown-t)

(defun store-token (token-type token-string)
                    ;(vector-push-extend (cons token-type token-string) *token-list*)
                    (setf *token-list* (cons (cons token-type token-string) *token-list*))
                    (setf *lexeme* (zero-array 'character)
                          *state* 'find-token
                          *type* 'unknown-t))
 
(defun lex (istream)
  (do
    ((c (read-char istream NIL)     ; Start with first char read
        (read-char istream NIL))    ; Read another char each step
     (*token-list* ())
     (*lexeme* (zero-array 'character))
     (*state* 'find-token)
     (*type* 'unknown-t))
    ((null c)                   ; End when c is null
     (setf *token-list* (cons (cons 'eof T) *token-list*))
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
               ((strmatch? *lexeme* '("true" "false")) 'boolean-ct)

               ;; Operators
               ((strmatch? *lexeme* '("and" "or"))              'binop-ot)
               ((strmatch? *lexeme* '("not" "sin" "cos" "tan")) 'unop-ot)

               ;; Primitives
               ((strmatch? *lexeme* '("bool"))   'boolean-pt)
               ((strmatch? *lexeme* '("int"))    'integer-pt)
               ((strmatch? *lexeme* '("real"))   'real-pt)
               ((strmatch? *lexeme* '("string")) 'string-pt)

               ;; Statements
               ((strmatch? *lexeme* '("if"))     'if-st)
               ((strmatch? *lexeme* '("while"))  'while-st)
               ((strmatch? *lexeme* '("let"))    'let-st)
               ((strmatch? *lexeme* '("stdout")) 'stdout-st)

               ;; Fallback
               ;; TODO: Differentiate functions
               (T 'variable-it))))

      ;; String
      ((char= c #\")
       (vector-pop *lexeme*)   ; Don't store the quotation mark

       ;; Set type
       (setf *type* 'string-ct)

       ;; Consume
       (do
         ((c (read-char istream NIL 'eof)
             (read-char istream NIL 'eof)))
         ((or (equal c 'eof)
              (char= c #\"))   ; Read until next quotation mark or EOF
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
                 ((strmatch? *lexeme* '(">=" "<=" "!=")) 'binop-ot)

                 ;; Assign statement
                 ((strmatch? *lexeme* '(":=")) 'assign-st)

                 ;; Unread and process singlechar ops
                 (T (unread-char c istream)
                    (vector-pop *lexeme*)
                    (cond
                      ((strmatch? *lexeme* '("(")) 'leftp-dt)
                      ((strmatch? *lexeme* '(")")) 'rightp-dt)
                      ((strmatch? *lexeme* '("!")) 'unknown-t)
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
