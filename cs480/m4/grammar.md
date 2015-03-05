Grammar Specification
=====================
S -> __(__ __)__ | __(__ S __)__ | SS | EXPRS

EXPRS -> OPERS | STMTS

EXPRLIST -> EXPRS | EXPRS EXPRLIST

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

IDLIST -> __(__ ids prims __)__ | __(__ ids prims __)__ IDLIST

Operators
---------
OPERS -> __( :=__ ids OPERS __)__ | __(__ binops OPERS OPERS __)__ | __(__ unops OPERS __)__ | constants | ids

binops -> __+__ | __-__ | __*__ | __/__ | __%__ | __^__ | __=__ | __>__ | __>=__ | __<__ | __<=__ | __!=__ | __or__ | __and__

unops -> __not__ | __sin__ | __cos__ | __tan__

Primitives
----------
prims -> __bool__ | __int__ | __real__ | __string__

Statements
----------
STMTS -> __(__ __if__ EXPRS EXPRS EXPRS __)__ | __(__ __if__ EXPRS EXPRS __)__ | __(__ __while__ EXPRS EXPRLIST __)__ | __(__ __let__ __(__ IDLIST __)__ __)__ | __(__ __stdout__ OPERS __)__
