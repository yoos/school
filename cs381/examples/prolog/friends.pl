:- include(likes).

friends(X,Y) :- likes(X,Z), likes(Y,Z).

marry(X,Y) :- likes(X,Y), likes(Y,X).
marry(X,Y) :- likes(X,Y), isRich(X).
marry(john,_).

isRich(bill).

