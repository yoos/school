\ Fibonnaci
\ UNFINISHED
: fib ( n -- )
  dup 0 <= if
    drop 1
  else
    dup 1 = if
      drop 1
    else
      dup 1 - recurse
      over 1 - recurse
    then
  then
;
bye
