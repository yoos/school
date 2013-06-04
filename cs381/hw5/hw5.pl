
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


