S -> __(__ __)__ | __(__ S __)__ | SS | expr

expr -> oper | stmts

exprlist -> expr | expr exprlist

oper -> __( :=__ id oper __)__ | __(__ binops oper oper __)__ | __(__ unops oper __)__ | constants | id

binops -> __+__ | __-__ | __*__ | __/__ | __%__ | __^__ | __=__ | __>__ | __>=__ | __<__ | __<=__ | __!=__ | __or__ | __and__

unops -> __-__ | __not__ | __sin__ | __cos__ | __tan__

constants -> bools | ints | reals | strings

bools -> __true__ | __false__

ints -> __regex for positive/negative ints in C__

reals -> __regex for positive/negative doubles in C__

strings -> __regex for str literal in C__ (any alphanumeric)

id -> __regex for ids in C__ (any lower and upper char or underscore followed by any combination of lower, upper, digits, or underscores)

idlist -> __(__ id prim __)__ | __(__ id prim __)__ idlist

prim -> __bool__ | __int__ | __real__ | __string__

stmts -> ifstmts | whilestmts | letstmts |printsmts

printstmts -> __(__ __stdout__ oper __)__

ifstmts -> __(__ __if__ expr expr expr __)__ | __(__ __if__ expr expr __)__

whilestmts -> __(__ __while__ expr exprlist __)__

letstmts -> __(__ __let__ __(__ varlist __)__ __)__
