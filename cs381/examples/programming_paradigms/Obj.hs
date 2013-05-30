-- 
-- Object-Oriented Language
--
module Obj where


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
          | New ID Fields Meths -- create new object
          | Msg ID Name         -- send message to object

type ID     = Int
type Fields = [(Name,Expr)]
type Meths  = [(Name,Stmt)]


instance Show Expr where
  show (N i)       = show i
  show (Plus e e') = showP e++"+"++showP e'
  show (Mult e e') = showP e++"*"++showP e'
  show (Var x)     = x

showP e@(N _)   = show e
showP e@(Var _) = show e
showP e         ="("++show e++")"

instance Show Stmt where
  show (Assg x e)    = x++":="++show e 
  show (Seq s s')    = show s++"; "++show s'
  show (While e s)   = "while "++show e++" {"++show s++"}"
  show (New i fs ms) = "new "++show i++"{"++showSt fs++" "++showSt ms++"}"
  show (Msg i m)     = show i++"<-"++m


-- Semantics
--
data Val = I Int  -- integer constants
         | Error  -- runtime error

instance Show Val where
  show (I i) = show i
  show Error = "error"


type State       = [(Name,Val)]
type Object      = (ID,State,Meths)
type Objects     = [Object]
type World       = (ID,Objects)
type Computation = World -> World

prWorld :: World -> String
prWorld (i,os) = show i++":"++concatMap showObj os

showObj :: Object -> String
showObj (i,fs,ms) = ' ':show i++"{"++showSt fs++" "++showSt ms++"}"

-- The semantics is defined by two function that map
-- expressions to values and statements to state
-- transitions.
--
evalE :: Expr -> State -> Val
evalE (N i)       _ = I i
evalE (Plus e e') s = bop (+) (evalE e s) (evalE e' s)
evalE (Mult e e') s = bop (*) (evalE e s) (evalE e' s)
evalE (Var x)     s = fromMaybe Error (lookup x s)


evalS :: Stmt -> Computation
evalS (Assg x e)    w = local w (\s->upd x (evalE e s) s)
evalS (Seq s1 s2)   w = evalS s2 (evalS s1 w)
evalS s@(While e b) w = case evalE e `inContextOf` w of
                          I 0   -> w
                          Error -> w
                          _     -> evalS (b `Seq` s) w
evalS (New i fs ms) w@(_,os) = (i,(i,fs',ms):os)
                             where fs'     = zip ns vs
                                   (ns,es) = unzip fs
                                   vs      = map (\e->evalE e []) es
evalS (Msg i m) w@(_,os) = case findObj i os of
                             Just (_,(_,_,ms),_) -> 
                               case lookup m ms of 
                                 Just s  -> evalS s (i,os)
                                 Nothing -> w
                             Nothing             -> w

inContextOf :: (State -> Val) -> World -> Val
inContextOf f (i,os) = case findObj i os of
                         Just (a,(_,s,_),b) -> f s
                         Nothing            -> Error

local :: World -> (State -> State) -> World
local (i,os) f  = case findObj i os of
                    Just (a,(_,s,m),b) -> (i,a++(i,f s,m):b)
                    Nothing            -> (i,os)

findObj :: ID -> Objects -> Maybe (Objects,Object,Objects)
findObj i [] = Nothing
findObj i ((j,s,m):os) | i==j      = Just ([],(j,s,m),os)
                       | otherwise = case findObj i os of
                                       Just (a,o,b) -> Just (a++[(j,s,m)],o,b)
                                       Nothing      -> Nothing

-- run :: Stmt -> World
-- run s = evalS s (0,[])
-- 
run :: Stmt -> IO ()
run s = putStrLn $ prWorld $ evalS s (0,[])


-- auxiliary functions
--
upd :: Name -> Val -> State -> State
upd x v [] = [(x,v)]
upd x v ((y,w):s) | x==y      = (x,v):s
                  | otherwise = (y,w):upd x v s

bop :: (Int -> Int -> Int) -> Val -> Val -> Val
bop b (I i) (I j) = I (i `b` j)
bop b _     _     = Error



-- "smart constructors" to simplify the construction
-- of syntax trees
-- 
[i1,i2,i3] = map N [1,2,3] 
[x,y,z,f]  = map Var ["x","y","z","f"]


-- example progams
--
cntSt  = [("x",N 0)] -- [("x",N 0 `Plus` N 0)]
cntMth = [("inc",Assg "x" (x `Plus` (N 1)))]

cnt = New 1 cntSt cntMth `Seq`
      New 2 cntSt cntMth `Seq`
      Msg 1 "inc" `Seq`
      Msg 2 "inc" `Seq`
      Msg 1 "inc" 

cnt' = New 1 cntSt cntMth `Seq`
       New 2 cntSt (cntMth++[("dec",Assg "x" (x `Plus` (N (-1))))]) `Seq`
       Msg 1 "inc" `Seq`
       Msg 2 "dec" `Seq`
       Msg 1 "inc" `Seq` 
       Msg 1 "dec" 

