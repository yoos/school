(load "tokens")

(defun syntax-parse (symbol-table grammar depth)
  (do
    ((sym (vector-pop symbol-table)
          (vector-pop symbol-table)))
    ((= (length symbol-table) 0))
    ()
    (let ((token-type (nth 0 sym))
          (token      (nth 1 sym)))
      (cond ((string= token "(")
             (syntax-parse symbol-table grammar depth))
            ((string= token ")")
             (return))
            (T
              (format T "~v~A~%" depth #\  token))
      ))))

