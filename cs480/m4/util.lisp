(defun strmatch? (str strlist)
  (if (null strlist)
    NIL
    (or (string= str (car strlist)) (strmatch? str (cdr strlist)))))

