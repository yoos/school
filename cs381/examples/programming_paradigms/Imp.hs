-- 
-- Imperative Language
--
module Imp where


import Data.Maybe
import STrace 


-- Abstract syntax
--
type Name = String

data Expr = N    Int            -- integer constant
          | Plus Expr Expr      -- addition
          | Mult Expr Expr      -- multiplication
          | Var  Name           -- reference to a variable

data Stmt = Assg Name Expr      -- assignment
          | Seq  Stmt Stmt      -- sequencing
          | While Expr Stmt     -- loop

instance Show Expr where
  show (N i)       = show i
  show (Plus e e') = showP e++"+"++showP e'
  show (Mult e e') = showP e++"*"++showP e'
  show (Var x)     = x

showP e@(N _)   = show e
showP e@(Var _) = show e
showP e         ="("++show e++")"

instance Show Stmt where
  show (Assg x e)  = x++":="++show e 
  show (Seq s s')  = show s++"; "++show s'
  show (While e s) = "while "++show e++" {"++show s++"}"


-- Semantics
--
data Val = I Int  -- integer constants
         | Error  -- runtime error

instance Show Val where
  show (I i) = show i
  show Error = "error"


type State = [(Name,Val)]

type Computation = State -> State

-- The semantics is defined by two functions that map
-- expressions to values and statements to state
-- transitions.
--
evalE :: Expr -> State -> Val
evalE (N i)       _ = I i
evalE (Plus e e') s = bop (+) (evalE e s) (evalE e' s)
evalE (Mult e e') s = bop (*) (evalE e s) (evalE e' s)
evalE (Var x)     s = fromMaybe Error (lookup x s)

evalS :: Stmt -> Computation
evalS (Assg x e)    s = upd x (evalE e s) s
evalS (Seq s1 s2)   s = evalS s2 (evalS s1 s)
evalS w@(While e b) s = case evalE e s of
                          I 0   -> s
                          Error -> s
                          _     -> evalS (b `Seq` w) s

run :: Stmt -> State
run s = evalS s []


-- auxiliary functions
--
upd :: Name -> Val -> State -> State
upd x v [] = [(x,v)]
upd x v ((y,w):s) | x==y      = (x,v):s
                  | otherwise = (y,w):upd x v s

bop :: (Int -> Int -> Int) -> Val -> Val -> Val
bop f (I i) (I j) = I (f i j)
bop f _     _     = Error



-- "smart constructors" to simplify the construction
-- of syntax trees
-- 
[i1,i2,i3,i4,i5] = map N [1..5] 
[x,y,z,f]  = map Var ["x","y","z","f"]


-- example progams
--
cpxy = Assg "x" i3 `Seq`
       Assg "y" x

fool = Assg "x" i1 `Seq`
       Assg "y" i3 `Seq`
       Assg "x" y  `Seq`
       Assg "y" x

swp = Assg "x" i1 `Seq`
      Assg "y" i3 `Seq`
      Assg "z" x  `Seq`
      Assg "x" y  `Seq`
      Assg "y" z

fac n = Assg "x" (N n) `Seq`
        Assg "y" (x `Plus` (N (-1))) `Seq`
        While y (Assg "x" (x `Mult` y) `Seq`
                 Assg "y" (y `Plus` (N (-1))))
      


-- tracing evaluator
--
evalT :: Stmt -> State -> Trace Stmt Val
evalT a@(Assg x e)    s = Tr [(a,upd x (evalE e s) s)] 
evalT a@(Seq s1 s2)   s = substStmt a (concatT t1 t2)
                          where t1 = evalT s1 s 
                                t2 = evalT s2 (getState t1)
evalT a@(While e b) s = case evalE e s of
                          I 0   -> Tr [(a,s)]
                          Error -> Tr [(a,s)]
                          _     -> evalT (assocR (b `Seq` a)) s

tr :: Stmt -> Trace Stmt Val
tr s = evalT s []

assocR ((x `Seq` y) `Seq` z) = x `Seq` (assocR y `Seq` assocR z)
assocR x = x

