;;; Define terminals
(defparameter constants   (list 'boolean-ct
                                'integer-ct
                                'real-ct
                                'string-ct))
(defparameter delimiters  (list 'leftp-dt
                                'rightp-dt))
(defparameter identifiers (list 'function-it
                                'variable-it))
(defparameter operators   (list 'binop-ot
                                'unop-ot))
(defparameter primitives  (list 'boolean-pt
                                'integer-pt
                                'real-pt
                                'string-pt))
(defparameter statements  (list 'stdout-st
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
(deftype token-t ()
  `(member ,@constants
           ,@delimiters
           ,@identifiers
           ,@operators
           ,@primitives
           ,@statements))
