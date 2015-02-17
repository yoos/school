Grammar Specification
=====================
S -> __(__ __)__ | __(__ S __)__ | SS | exprs

exprs -> opers | stmts

exprlist -> exprs | exprs exprlist

Constants
---------
constants -> bools | ints | reals | strings

bools -> __true__ | __false__

ints -> __regex for positive/negative ints in C__

reals -> __regex for positive/negative doubles in C__

strings -> __regex for str literal in C__ (any alphanumeric)

Identifiers
-----------
ids -> __regex for ids in C__ (any lower and upper char or underscore followed by any combination of lower, upper, digits, or underscores)

idlist -> __(__ ids prims __)__ | __(__ ids prims __)__ idlist

Operators
---------
opers -> __( :=__ ids opers __)__ | __(__ binops opers opers __)__ | __(__ unops opers __)__ | constants | ids

binops -> __+__ | __-__ | __*__ | __/__ | __%__ | __^__ | __=__ | __>__ | __>=__ | __<__ | __<=__ | __!=__ | __or__ | __and__

unops -> __not__ | __sin__ | __cos__ | __tan__

Primitives
----------
prims -> __bool__ | __int__ | __real__ | __string__

Statements
----------
stmts -> ifstmts | whilestmts | letstmts | printstmts

printstmts -> __(__ __stdout__ opers __)__

ifstmts -> __(__ __if__ exprs exprs exprs __)__ | __(__ __if__ exprs exprs __)__

whilestmts -> __(__ __while__ exprs exprlist __)__

letstmts -> __(__ __let__ __(__ idlist __)__ __)__
