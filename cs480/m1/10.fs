\ Int to double converter
\ d is data stack. floats are actually doubles in Forth.
: convertint ( s -- d )
  s>d
  ;

123 convertint d.s
bye
