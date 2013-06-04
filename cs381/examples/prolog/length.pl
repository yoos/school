len([],   0).
len([_|L],N) :- len(L,M), N is M+1.

len2([],   0).
len2([_|L],N) :- N is M+1, len2(L,M).

len3([],   0).
len3([_|L],N) :- M is N-1, len3(L,M).

len4([],   0).
len4([_|L],N) :- len4(L,M), N = M+1.



