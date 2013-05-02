--
-- Examples illustrating the definition of functions to
-- generate syntax trees
--
module GenExpr where

import ExprSyn


-- genSum generates an expression, i.e. syntax tree,
-- to compute the sum of the first n integers.
-- 
-- The base case is for n=1. In this case we simply generate
-- the expression representing that integer, namely N 1.
-- Otherwise, we genereate a Plus expression whose first 
-- argument is an expression representing the current number n
-- and whose second argument is the expression representing the
-- sum of numbers from 1 to n-1, which is obtained through a
-- recursive call.
-- 
genSum :: Int -> Expr
genSum 1 = N 1
genSum n = Plus (N n) (genSum (n-1))

{-

Example:

*GenExpr> genSum 4
Plus (N 4) (Plus (N 3) (Plus (N 2) (N 1)))

-}

-- NOTE: The above function does not terminate for
-- inputs < 1. This could be handled by adding a guard to 
-- the second case together with a default result.
--
genSum' :: Int -> Expr
genSum' 1 = N 1
genSum' n | n>1       = Plus (N n) (genSum (n-1))
          | otherwise = N 0

{-

Example:

*GenExpr> genSum' (-3)
N 0

-}


