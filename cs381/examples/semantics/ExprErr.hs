-- 
-- Semantics of arithmetic expressions with potential errors
--
module ExprSem where


data Expr = N Int
          | Plus Expr Expr
          | Neg Expr
          | Div Expr Expr
          deriving Show

-- Arithmetic expressions are evaluated to Int.
-- To account for errors, the semantic domain is
-- defined as Maybe Int.
-- 
type D = Maybe Int


-- The semantics is defined as a function that maps
-- expressions of type Expr to D. Normal results (i.e.
-- integers) are wrapped by the Just constructor to
-- be of type D. Just 'injects' integers into the
-- Maybe type. Error values are represented by the
-- constructor Nothing.
-- 
-- sem :: Expr -> D
sem :: Expr -> Maybe Int
sem (N i)       = Just i
sem (Neg e)     = case sem e of
                    Just i  -> Just (-i)
                    _       -> Nothing
--                     Nothing -> Nothing
sem (Plus e e') = case (sem e,sem e') of
                    (Just i,Just j) -> Just (i+j)
                    _               -> Nothing
sem (Div e e')  = 
     case (sem e,sem e') of
          (Just i,Just j) -> if j/=0 then Just (i `div` j) 
                                     else Nothing
          _               -> Nothing


-- examples
--
e1 = Plus (N 3) (N 4)

e2 = Div (N 21) e1

e3 = Div e2 (Plus e1 (Neg (N 7)))

