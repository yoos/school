-- 
-- Expression Language with Nested Pairs
--
module ExprNPair where

data Expr = N Int
          | Pair Expr Expr
	  | Fst Expr
	  | Swap Expr
          deriving Show

data Type = Int | PairT Type Type | TypeError
            deriving (Eq, Show)
 

-- Some examples expressions are:
--
a = N 1
b = Pair a (N 2)
c = Pair a b
d = Swap c
e = Fst b
f = Fst (Fst d)
g = Fst a
h = Swap a
j = Pair h (N 3)
k = Swap j

ex = [a,b,c,d,e,f,g,h,j,k]

-- A data type to represent the union type of
-- semantic values
-- 
data Val = I Int
         | P Val Val
         | Error
         deriving Show

-- Semantics maps into union type
-- No error handling yet.
--
sem :: Expr -> Val
sem (N i)       = I i
sem (Pair e e') = P (sem e) (sem e')  -- Note: Lazy Evaluation!
sem (Fst e)     = case sem e of
                     (P v _) -> v
                     _       -> Error
sem (Swap e)    = case sem e of
                     P v w -> P w v   -- Note: Lazy Evaluation!
                     _     -> Error


psem :: Expr -> String
psem e = show e++" ==> "++show (sem e)++"\n"

semAll :: IO ()
semAll = putStrLn $ concatMap psem ex


-- type checker
--
tc :: Expr -> Type
tc (N i)       = Int
-- tc (Pair e e') = PairT (tc e) (tc e')
tc (Pair e e') | ok t && ok u = PairT t u
               | otherwise    = TypeError
                 where (t,u) = (tc e,tc e')
tc (Fst e)     = case tc e of
                   PairT t _ -> t
                   _         -> TypeError
tc (Swap e)    = case tc e of
                   PairT t u -> PairT u t
                   _         -> TypeError

ok :: Type -> Bool
ok TypeError = False
ok _         = True


ptc :: Expr -> String
ptc e =  show e++" :: "++show (tc e)++"\n"

tcAll :: IO ()
tcAll = putStrLn $ concatMap ptc ex


