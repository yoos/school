\ For loop
create i
: myfor 6 0 do
  5 i @ - .        \ Print (5-i)
  i @ 1 - i !      \ Decrement i
  loop ;

5 i !   \ Init i later because race condition weirdness?
myfor
bye
