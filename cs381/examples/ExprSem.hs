-- 
-- Semantics of arithmetic expressions
--
module ExprSem where

-- import ExprSyn

data Expr = N Int
          | Plus Expr Expr
          | Neg Expr
          | Mult Expr Expr
          deriving Show

-- The semantics is defined as a function that maps
-- expressions of type Expr to integers.
--
sem :: Expr -> Int
sem (N i)       = i
sem (Plus e e') = sem e + sem e'
sem (Mult e e') = sem e * sem e'
sem (Neg e)     = -(sem e)

e = Neg (N 5) `Plus` N 7

