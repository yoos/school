-- 
-- Two-Type Expression Language
--
module Expr2Unsafe where

data Expr = N Int
          | Plus Expr Expr
	      | Equal Expr Expr
	      | Not Expr
          deriving Show


-- Some examples expressions are:
--
x = N 2
y = Plus (N 3) (Plus (N 4) (N 5))
z = Not (Equal (Plus (N 1) (N 1)) (N 2))

false = N 1 `Equal` N 2
fakeTrue = N 2

e1 = Not fakeTrue
e2 = e1 `Plus` N 1
e3 = fakeTrue `Equal` Not (Not fakeTrue)


-- Unsafe evaluator maps into integers
--
eval :: Expr -> Int
eval (N i)        = i
eval (Plus e e')  = eval e+eval e'
eval (Equal e e') | eval e == eval e' = 1
                  | otherwise         = 0
eval (Not e)      | eval e==0 = 1
                  | otherwise = 0

