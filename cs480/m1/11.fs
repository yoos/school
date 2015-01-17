\ Factorial
: fact ( n -- n! )
  dup
  0 <= if drop 1 else dup 1 - recurse * then
  ;

6 fact .
bye
