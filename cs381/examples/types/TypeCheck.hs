-- 
-- Type Checker for the Expression Language
--
module TypeCheck where

import Expr2


-- Expr data type and examples are imported from module Expr2
-- 
-- data Expr = N Int
-- 	  | Plus Expr Expr
-- 	  | Equal Expr Expr
-- 	  | Not Expr
--    deriving Show
-- 
-- x = N 2
-- y = Plus (N 3) (Plus (N 4) (N 5))
-- z = Not (Equal (Plus (N 1) (N 1)) (N 2))
-- typeError = Not (N 2)
-- e1 = N 2 `Equal` N 2
-- e2 = N 1 `Equal` e1
-- e3 = Not (N 2 `Plus` N 2)

-- A data type to represent types
--
data Type = Int | Bool | TypeError
            deriving (Eq,Show)

-- In addition to deriving Show for printing types, we add
-- a "deriving Eq" clause to be able to compare Type values
-- for equality.


-- The type checker uses guards to constrain the selection
-- of definitions by conditions about the type of subexpressions.
--
tc :: Expr -> Type
tc (N i)                                    = Int
tc (Plus e e')  | tc e==Int  && tc e'==Int  = Int
tc (Equal e e') | tc e==Int  && tc e'==Int  = Bool
                | tc e==Bool && tc e'==Bool = Bool
tc (Not e)      | tc e==Bool                = Bool
tc _                                        = TypeError


-- We can use the type checker to perform only safe evaluations
--
typeSafe :: Expr -> Bool
typeSafe e = tc e /= TypeError

evalStatTC :: Expr -> Maybe Val
evalStatTC e | typeSafe e = Just (eval e)
             | otherwise  = Nothing


-- compare "evalStatTC e2" with "eval e2"




