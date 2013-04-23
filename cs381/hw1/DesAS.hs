module DesAS where

data Expr = N Int
          | Plus Expr Expr
          | Times Expr Expr
          | Neg Expr
          deriving Show

data Op = Add | Multiply | Negate
        deriving Show

data Exp = Num Int
         | Apply Op [Exp]
         deriving Show


altExp = Apply Multiply [Apply Negate [Apply Add [Num 3, Num 4]], Num 7]


-- vim: expandtab

