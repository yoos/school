contains(house,bathroom).
contains(house,kitchen).
contains(kitchen,fridge).
contains(fridge,mouse).


/* The order of rules does not matter,
   but the order of subgoals affects termination. */

inside(X,Y) :- contains(Y,X).
inside(X,Y) :- contains(Y,Z), inside(X,Z).

