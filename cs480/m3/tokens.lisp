(defpackage #:lexer-tokens)

;;; Define type members
(defparameter constants '(:boolean-ct
                           :integer-ct
                           :real-ct
                           :string-ct))
(defparameter identifiers '(:function-it
                             :variable-it))
(defparameter primitives '(:boolean-pt
                            :integer-pt
                            :real-pt
                            :string-pt))
(defparameter statements '(:stdout-st
                            :if-st
                            :while-st
                            :let-st
                            :assign-st))

;;; Define types
(deftype constant-t   () `(member ,@constants))
(deftype identifier-t () `(member ,@identifiers))
(deftype primitive-t  () `(member ,@primitives))
(deftype statement-t  () `(member ,@statements))
(deftype op-t         ())
(deftype unknown-t    ())

;;; Encapsulate the above types as a token type
;;; TODO(yoos): don't think this is quite right.
(deftype token-t ()
  `(member :constant-t
           :identifier-t
           :op-t
           :primitive-t
           :statement-t
           :unknown-t))

