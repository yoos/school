-- 
-- Expression Language with Pairs
--
module ExprPair where

data Expr = N Int
          | Pair Expr Expr
	  | Fst Expr
	  | Swap Expr
          deriving Show

data Type = Int | IPair | TypeError
            deriving (Eq, Show)
 

-- Some examples expressions are:
--
a = N 1
b = Pair a (N 2)
c = Fst b
d = Swap b
e = Swap (Swap d)
f = Fst e
g = Fst a
h = Fst (Fst d)
i = Pair b b

ex = [a,b,c,d,e,f,g,h,i]

-- A data type to represent the union type of
-- semantic values
-- 
data Val = I Int
         | P Int Int
         | Error
         deriving Show

-- Semantics maps into union type
-- No error handling yet.
--
sem :: Expr -> Val
sem (N i)       = I i
sem (Pair e e') = case (sem e,sem e') of
                     (I i,I j) -> P i j
                     _         -> Error
sem (Fst e)     = case sem e of
                     (P i _) -> I i
                     _       -> Error
sem (Swap e)    = case sem e of
                     P i j -> P j i
                     _     -> Error


psem :: Expr -> String
psem e = show e++" ==> "++show (sem e)++"\n"

semAll :: IO ()
semAll = putStrLn $ concatMap psem ex


-- type checker
--
tc :: Expr -> Type
tc (N i)                                  = Int
tc (Pair e e') | tc e==Int  && tc e'==Int = IPair
tc (Fst e)     | tc e==IPair              = Int
tc (Swap e)    | tc e==IPair              = IPair 
tc _                                      = TypeError

ptc :: Expr -> String
ptc e =  show e++" :: "++show (tc e)++"\n"

tcAll :: IO ()
tcAll = putStrLn $ concatMap ptc ex

typeSafe :: Expr -> Bool
typeSafe e = tc e /= TypeError

semTC :: Expr -> Maybe Val
semTC e | typeSafe e = Just (sem e)
        | otherwise  = Nothing

psemTC :: Expr -> String
psemTC e = show e++" ==> "++show (semTC e)++"\n"

semTCAll :: IO ()
semTCAll = putStrLn $ concatMap psemTC ex


