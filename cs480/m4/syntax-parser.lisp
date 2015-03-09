(load "tokens")

(defparameter *oldsym* ())
(defparameter *sym* ())
(defparameter *symbol-table* ())
(defparameter *depth* 0)

;;; Convenience function so popping from symbol table fails gracefully.
(defun table-pop (symbol-table)
  (if (> (fill-pointer symbol-table) 0)
    (vector-pop symbol-table)))

(defun get-sym ()
  (defparameter *oldsym* *sym*)
  (setf *sym* (table-pop *symbol-table*))
  (if (null *sym*)
    (setf *sym* (cons 'ACCEPT T))))

(defun unget-sym ()
  (format T "Ungetting ~S.. " *sym*)
  (vector-push-extend *sym* *symbol-table*)
  (defparameter *sym* *oldsym*)
  (format T "sym now ~S~%" *sym*)
  )

(defun parse-err (str)
  (format T "~,,v,@A on ~S~%" (* *depth* 2) (format NIL "[ERROR] ~A" str) *sym*)
  )

(defun parse-info (str)
  (format T "~,,v,@A" (* 2 *depth*) (format NIL "[INFO] ~A" str))
  )

(defun accept (token-type)
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Trying to accept ~S on ~S.. " token-type (car *sym*)))
  (setf res
        (cond ((equal (car *sym*) token-type)
               (format T "good!~%")
               (get-sym)
               T)
              (T
                (format T "wrong.~%")
                NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun expect (token-type)
  (cond ((accept token-type) T)
        (T
          (parse-err (format NIL "Unexpected symbol - expected ~S" token-type)) NIL)))

(defun parse (symbol-table)
  (setf *symbol-table* symbol-table)
  (get-sym)
  (setf res
        (cond ((parse-S))
              (T (parse-err "Error while parsing") NIL)))
  res
  )

(defun parse-S ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing S on ~S~%" *sym*))
  (if (equal (car *sym*) 'ACCEPT)
    T
    (setf res
          (cond ((parse-+S))
                ((parse-_S))
                (T (parse-err "Error while parsing S") NIL))))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-+S ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing +S on ~S~%" *sym*))
  (setf res
        (cond ((parse-_S)
               (parse-S ))
              (T (parse-err "Error while parsing +S") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-_S ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing _S on ~S~%" *sym*))
  (setf res
        (cond ((parse-nil))
              ((accept 'leftp-dt)
               (parse-S)
               (expect 'rightp-dt))
              ((parse-expr))
              (T (parse-err "Error while parsing _S") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-nil ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing nil on ~S~%" *sym*))
  (setf res
        (cond ((accept 'leftp-dt)
               (expect 'rightp-dt))
              (T (parse-err "Error while parsing nil") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-expr ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing expr on ~S~%" *sym*))
  (setf res
        (cond ((parse-oper))
              ((parse-stmt))
              (T (parse-err "Error while parsing expr") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-oper ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing oper on ~S~%" *sym*))
  (setf res
        (cond ((accept 'leftp-dt)
               (parse-Poper)
               (expect 'rightp-dt))
              ((parse-const))
              ((parse-id))
              (T (parse-err "Error while parsing oper") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-Poper ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing Poper on ~S~%" *sym*))
  (setf res
        (cond ((accept 'assign-st)
               (parse-id)
               (parse-oper))
              ((accept 'binop-ot)
               (parse-oper)
               (parse-oper))
              ((accept 'unop-ot)
               (parse-oper))
              (T (parse-err "Error while parsing Poper") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-stmt ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing stmt on ~S~%" *sym*))
  (setf res
        (cond ((accept 'leftp-dt)
               (parse-Pstmt)
               (expect 'rightp-dt))
              (T (parse-err "Error while parsing stmt") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-Pstmt ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing Pstmt on ~S~%" *sym*))
  (setf res
        (cond ((parse-ifstmt))
              ((parse-elifstmt))
              ((accept 'while-st)
               (parse-expr)
               (parse-exprs))
              ((accept 'let-st)
               (expect 'leftp-dt)
               (parse-ids)
               (expect 'rightp-dt))
              ((accept 'stdout-st)
               (parse-oper))
              (T (parse-err "Error while parsing Pstmt") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-ifstmt ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing ifstmt on ~S~%" *sym*))
  (setf res
        (cond ((accept 'if-st)
               (parse-expr)
               (parse-expr))
              (T (parse-err "Error while parsing ifstmt") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-elifstmt ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing elifstmt on ~S~%" *sym*))
  (setf res
        (cond ((accept 'if-st)
               (parse-expr)
               (parse-expr)
               (parse-expr))
              (T (parse-err "Error while parsing elifstmt") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-exprs ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing exprs on ~S~%" *sym*))
  (setf res
        (cond ((parse-+exprs))
              ((parse-expr))
              (T (parse-err "Error while parsing exprs") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-+exprs ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing +exprs on ~S~%" *sym*))
  (setf res
        (cond ((parse-expr)
               (parse-exprs))
              (T (parse-err "Error while parsing +exprs") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-ids ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing ids on ~S~%" *sym*))
  (setf res
        (cond ((accept 'leftp-dt)
               (parse-id)
               (parse-prim)
               (expect 'rightp-dt))
              ((parse-+ids))
              (T (parse-err "Error while parsing ids") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-+ids ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing +ids on ~S~%" *sym*))
  (setf res
        (cond ((accept 'leftp-dt)
               (parse-id)
               (parse-prim)
               (expect 'rightp-dt)
               (parse-ids))
              (T (parse-err "Error while parsing +ids") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-const ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing const on ~S~%" *sym*))
  (setf res
        (cond ((accept 'boolean-ct))
              ((accept 'integer-ct))
              ((accept 'real-ct))
              ((accept 'string-ct))
              (T (parse-err "Error while parsing const") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-id ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing id on ~S~%" *sym*))
  (setf res
        (cond ((accept 'function-it))
              ((accept 'variable-it))
              (T (parse-err "Error while parsing id") NIL)))
  (setf *depth* (- *depth* 1))
  res)

(defun parse-prim ()
  (setf *depth* (+ *depth* 1))
  (parse-info (format NIL "Parsing prim on ~S~%" *sym*))
  (setf res
        (cond ((accept 'boolean-pt))
              ((accept 'integer-pt))
              ((accept 'real-pt))
              ((accept 'string-pt))
              (T (parse-err "Error while parsing prim") NIL)))
  (setf *depth* (- *depth* 1))
  res)




(defun syntax-parse (symbol-table grammar depth)
  (let ((parse-tree (list (cons 'leftp-dt "("))))
    (do
      ((sym (table-pop symbol-table)
            (table-pop symbol-table)))
      ((null sym))
      (let ((token-type (car sym))
            (token      (cdr sym)))
        (cond ((equal token-type 'rightp-dt)   ;; Handle closing parenthesis specially
               (format T "~,,v,@A~%" (* 2 (- depth 1)) token)
               (return))
              ((equal token-type 'leftp-dt)   ;; Recurse on opening parenthesis
               (format T "~,,v,@A~%" (* 2 depth) token)
               (let ((subtree (syntax-parse symbol-table grammar (+ 1 depth))))
                 (setf parse-tree
                       (cons subtree parse-tree))))
              (T
                ;; Print to screen at correct indentation
                ;; See http://stackoverflow.com/questions/20072959/lisp-format-a-character-a-number-of-times
                (format T "~,,v,@A~%" (* 2 depth) token)
                (setf parse-tree
                      (cons sym parse-tree))
                ))))
    (setf parse-tree
          (cons (cons 'rightp-dt ")") parse-tree))
    (nreverse parse-tree)))
