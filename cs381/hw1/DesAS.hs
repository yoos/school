{--- Designing Abstract Syntax ---}

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


origExp = Times (Neg (Plus (N 3) (N 4))) (N 7)


{- 3a: Alternative expression -}

altExp = Apply Multiply [Apply Negate [Apply Add [Num 3, Num 4]], Num 7]


{- 3b: Advantages/disadvantages -}

-- The alternate expression has no contraint on how many operands an operator
-- can be applied to, which makes it more concise when there are more than two
-- operands. On the flipside, because there is no constraint, it is possible to
-- apply an operator such as Add to only a single or no operand, which makes no
-- sense. The flexibility of the alternative representation allows the user to
-- more freely express expressions but also leaves it up to the user to do so
-- properly.


{- 3c: Translator -}

translate :: Expr -> Exp
translate (N a) = Num a
translate (Plus a b) = Apply Add [translate a, translate b]
translate (Times a b) = Apply Multiply [translate a, translate b]
translate (Neg a) = Apply Negate [translate a]


-- vim: expandtab

