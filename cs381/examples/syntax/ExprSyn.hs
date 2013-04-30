-- 
-- Arithmetic expressions 
--
module ExprSyn where


-- The Expr grammar is:
-- 
--   Expr ::=  Int | Expr + Expr | -Expr
--
-- This grammar consists of just one nonterminal
-- and three productions. 
-- 
-- Grammars are represented as Haskell data types as follows.
--
-- (1) For each nonterminal define a data type
-- (2) For each production, define a constructor 
-- (3) Argument types of constructors are determined by the 
--     production's nonterminals
-- (4) For nonterminals that represent tokens (like Id, Con, 
--     or Num) use built-in Haskell types (like String or Int)


data Expr = N Int
          | Plus Expr Expr
          | Neg Expr
          deriving Show

-- Add "deriving Show" to be able to print values in GHC
--


-- Some examples expressions are:
--
x = N 2
y = Plus (N 3) (Neg (Plus (N 4) (N 5)))
z = Plus (Neg (Neg (N 4))) (N 1)

