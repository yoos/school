(load "config")

(defparameter *depth* 0)

(defun strmatch? (str strlist)
  (if (null strlist)
    NIL
    (or (string= str (car strlist)) (strmatch? str (cdr strlist)))))

(defun parse-reject (str)
  (format *enable-debug* "~,,v,@A" (* *depth* 2) (format NIL "[REJECT ~S] ~@?" *depth* str))
  NIL)

(defun parse-info (str &optional (enable-indent T))
  (if enable-indent
    (format *enable-debug* "~,,v,@A" (* 2 *depth*) (format NIL "[INFO ~S] ~@?" *depth* str))
    (format *enable-debug* "~@?" str))
  T)

;;; Flatten nested list
(defun flatten (ls)
  (labels ((mklist (x) (if (listp x) x (list x))))
    (mapcan #'(lambda (x) (if (atom x) (mklist x) (flatten x))) ls)))
