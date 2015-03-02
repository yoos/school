(defpackage #:lexer-tokens)

;;; Define terminals
(defparameter constants `('boolean-ct
                          'integer-ct
                          'real-ct
                          'string-ct))
(defparameter delimiters `('leftp-dt
                           'rightp-dt
                           'semicolon-dt))   ; TODO(yoos)' Do we really need to categorize these separately?
(defparameter identifiers `('function-it
                            'variable-it))
(defparameter operators `('binop-ot
                          'unop-ot))
(defparameter primitives `('boolean-pt
                           'integer-pt
                           'real-pt
                           'string-pt))
(defparameter statements `('stdout-st
                           'if-st
                           'while-st
                           'let-st
                           'assign-st))

;;; Define categorical symbols
(deftype constant-t   () `(member ,@constants))
(deftype delimiter-t  () `(member ,@delimiters))
(deftype identifier-t () `(member ,@identifiers))
(deftype operator-t   () `(member ,@operators))
(deftype primitive-t  () `(member ,@primitives))
(deftype statement-t  () `(member ,@statements))
(deftype unknown-t    ())

;;; Encapsulate the above symbols as a token type
;;; TODO(yoos): don't think this is quite right.
(deftype token-t ()
  `(member 'constant-t
           'delimiter-t
           'identifier-t
           'operator-t
           'primitive-t
           'statement-t
           'unknown-t))

