likes(mary,X) :- animal(X), not(snake(X)).

animal(dog).
animal(python).

snake(python).




/*

The built-in negation predicate "not" is defined
as follows. We are using "noT" to avoid a conflict
with the built-in predicate.

*/

noT(P) :- P, !, fail.
noT(P).

/*

Try several predicates and their negation

noT(3=4).

noT(3=3).

noT(X=4).

noT(fail).

*/


mother(mary).
mother(eve).

hadsex(eve).

/*

Note that the position of the negated predicate matters!

*/

virginBirth(X) :- mother(X), noT(hadsex(X)).

virginBirth2(X) :- noT(hadsex(X)), mother(X).

/*

Why doesn't the goal "virginBirth2(X)" produce any result?
Because X is not bound. Formally, unbound variables are
existentially quantified. Therefore, in the second definition,

  noT(hadsex(X)) 
  
expands to:

  not (\exists X. hadsex(X))

which is the same as:

  \forall X. not(hadsex(X))

However, that goal cannot be satisfied since there is one fact 
about hadsex. Removing that fact would lead to an 
'undefined predicate' error.

Bottom line: noT works well if all variables used are
instantiated, but it might result in unexpected behavior
if it involves free variables.

*/
