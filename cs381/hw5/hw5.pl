
/* Exercise 1 */

when(275,10).
when(261,12).
when(381,11).
when(398,12).
when(399,12).

where(275,owen102).
where(261,dear118).
where(381,cov216).
where(398,dear118).
where(399,cov216).

enroll(mary,275).
enroll(john,275).
enroll(mary,261).
enroll(john,381).
enroll(jim,399).

/* a */
schedule(S, P, T) :- enroll(S, C), when(C, T), where(C, P).

/* b */
usage(P, T) :- when(C, T), where(C, P).

/* c */
conflict(X, Y) :- where(X, P), where(Y, P), when(X, T), when(Y, T), X \= Y.

/* d */
meet(X, Y) :- schedule(X, P, T), schedule(Y, P, S), X \= Y, S is T+1.


/* Exercise 2 */

/* a */
rdup([], []).
rdup([X,X|Y], M) :- rdup([X|Y], M), !.
rdup([X|Y], [X|Z]) :-  rdup(Y, Z).

/* b */
flat([], []).
flat([[]|Y], L) :- flat(Y, L).
flat([[X|Y]|Z], L) :- flat([X,Y|Z], L), !.
flat([X|Y], [X|Z]) :- flat(Y, Z).

/* c */
decrement([], []).
decrement([A|B], [C|D]) :- decrement(B, D), C is A-1.

project([], _, []).
project(_, [], []).
project([X|Y], [A|B], [A|Z]) :- decrement(Y, N), project(N, B, Z), X is 1.
project([X|Y], [_|B], L)     :- decrement([X|Y], N), project(N, B, L), not(X is 1).

