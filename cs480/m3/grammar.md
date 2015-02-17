S -> _(_ _)_ | _(_S_)_ | SS | expr
expr -> oper | stmts
exprlist -> expr | expr exprlist

oper -> _( :=_ id oper _)_ | _(_ binops oper oper _)_ | _(_ unops oper _)_ | constants | id
binops -> _+_ | _-_ | _*_ | _/_ | _%_ | _^_ | _=_ | _>_ | _>=_ | _<_ | _<=_ | _!=_ | _or_ | _and_
unops -> _-_ | _not_ | _sin_ | _cos_ | _tan_

constants -> bools | ints | reals | strings
bools -> _true_ | _false_
ints -> _regex for positive/negative ints in C_
reals -> _regex for positive/negative doubles in C_
strings -> _regex for str literal in C_ (any alphanumeric)

id -> _regex for ids in C_ (any lower and upper char or underscore followed by any combination of lower, upper, digits, or underscores)
idlist -> _(_ id prim _)_ | _(_ id prim _)_ idlist

prim -> _bool_ | _int_ | _real_ | _string_

stmts -> ifstmts | whilestmts | letstmts |printsmts
printstmts -> _(_ _stdout_ oper _)_
ifstmts -> _(_ _if_ expr expr expr _)_ | _(_ _if_ expr expr _)_
whilestmts -> _(_ _while_ expr exprlist _)_
letstmts -> _(_ _let_ _(_ varlist _)_ _)_
