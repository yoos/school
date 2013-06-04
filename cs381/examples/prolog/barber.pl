
shaves(barber,X) :- male(X), \+(shaves(X,X)).

male(barber). 

