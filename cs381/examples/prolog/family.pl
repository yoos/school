male(adam).
male(cain).
male(abel).
male(seth).
male(enoch).

female(eve).
female(sisCain).
female(sisAbel).

parent(adam,cain).
parent(adam,abel).
parent(adam,seth).
parent(adam,sisCain).
parent(adam,sisAbel).
parent(eve,cain).
parent(eve,abel).
parent(eve,seth).
parent(eve,sisCain).
parent(eve,sisAbel).
parent(cain,enoch).
parent(sisAbel,enoch).


father(X,Y)      :- parent(X,Y), male(X).
mother(X,Y)      :- parent(X,Y), female(X).
grandfather(X,Y) :- father(X,Z), parent(Z,Y).
grandmother(X,Y) :- mother(X,Z), parent(Z,Y).

child(X,Y)    :- parent(Y,X).
son(X,Y)      :- child(X,Y), male(X).
daughter(X,Y) :- child(X,Y), female(X).


/* */


grandchild(X,Y)    :- child(X,Z), child(Z,Y).
grandson(X,Y)      :- grandchild(X,Y), male(X).
granddaughter(X,Y) :- grandchild(X,Y), female(X).


/* */


ancestor(X,Y) :- parent(X,Y).
ancestor(X,Y) :- parent(X,Z), ancestor(Z,Y).


/* will not terminate after last response */
ancestor2(X,Y) :- parent(X,Y).
ancestor2(X,Y) :- ancestor2(Z,Y), parent(X,Z).


/* will not terminate at all */
ancestor3(X,Y) :- ancestor3(Z,Y), parent(X,Z).
ancestor3(X,Y) :- parent(X,Y).


/* */


sibling(X,Y) :- parent(Z,X), parent(Z,Y), X\=Y.
brother(X,Y) :- sibling(X,Y), male(X).
sister(X,Y) :- sibling(X,Y), female(X).


/* */


aunt(X,Y) :- parent(Z,Y), sister(X,Z), \+ parent(X,Y).
uncle(X,Y) :- parent(Z,Y), brother(X,Z), \+ parent(X,Y).


/* */


mate(X,Y) :- parent(X,Z), parent(Y,Z), X\=Y.

incest(X,Y) :- mate(X,Y), sibling(X,Y).

