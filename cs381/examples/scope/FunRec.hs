-- 
-- Extending arithmetic expressions by functions
--
module FunRec where

import Trace 


type Name = String

data Expr = N Int                -- integer constant
          | Plus Expr Expr       -- addition
          | Mult Expr Expr       -- multiplication
          | IfZ Expr Expr Expr   -- conditional
          | Fun Name Expr        -- function abstraction
          | Var Name             -- reference to a variable
          | App Expr Expr        -- application
          | Let Name Expr Expr   -- local definition
          | Rec Name Expr Expr   -- recursive definition


instance Show Expr where
  show (N i)        = show i
  show (Plus e e')  = showP e++"+"++showP e'
  show (Mult e e')  = showP e++"*"++showP e'
  show (IfZ c e e') = "if "++show c++" then "++show e++" else "++show e'
  show (Fun x e)    = showFun x e
  show (Var x)      = x
  show (App e e')   = case e of 
                        App _ _ -> show e++" "++showP e'
                        _       -> showP e++" "++showP e'
  show (Let x e e') = "let "++x++"="++show e++" in "++show e'
  show (Rec x e e') = "rec "++x++"="++show e++" in "++show e'

showFun x e = "\\"++x++"->"++show e

showClos x e s = showFun x e++";@"++show (length s)

showP e@(N _)   = show e
showP e@(Var _) = show e
showP e         ="("++show e++")"

data Val = I Int              -- integer constants
         | C Name Expr Stack  -- closures (stack remembers values from definition)
         | Error

instance Show Val where
  show (I i)     = show i
  show (C x e s) = showClos x e s
  show Error     = "error"

isZero :: Val -> Bool
isZero (I 0) = True
isZero _     = False

isClos :: Val -> Bool
isClos (C _ _ _) = True
isClos _         = False

type Stack = [(Name,Val)]


-- The semantics are defined as a function that maps
-- expressions of type Expr to values. Values are
-- integers, functions, or errors. Note that eval
-- takes as an additional argument a runtime stack
-- for storing local definitions of variables.
--
eval :: Stack -> Expr -> Val
eval _ (N i)        = I i
eval s (Plus e e')  = bop (+) (eval s e) (eval s e')
eval s (Mult e e')  = bop (*) (eval s e) (eval s e')
eval s (IfZ c e e') | isZero (eval s c) = eval s e
                    | otherwise         = eval s e'
eval s (Fun x e)    = C x e s 
eval s (Var x)      = getVar x s
eval s (App f e')   = case eval s f of
                        C x e s' -> eval ((x,eval s e'):s') e
                        _        -> Error
eval s (Let x e e') = eval ((x,eval s e):s) e'
eval s (Rec x e e') = eval s' e'
                      where s' = (x,eval s' e):s


bop :: (Int -> Int -> Int) -> Val -> Val -> Val
bop b (I i) (I j) = I (i `b` j)
bop b _     _     = Error

getVar :: Name -> Stack -> Val
getVar x s = case lookup x s of
               Just v  -> v
               Nothing -> Error


-- "smart constructors" to simplify the construction
-- of syntax trees
-- 
n1         = N (-1)
[i1,i2,i3] = map N [1,2,3] 
[x,y,z,f]  = map Var ["x","y","z","f"]


-- example expressions
--
--- functions
suc = Fun "y" (y `Plus` i1)
dbl = Fun "x" (x `Plus` x)
--- applications
ds3 = App dbl (App suc i3)
--- definitions
letxy = Let "x" i1 (Let "y" i2 (x `Plus` y))
letxy' = Let "x" i1 ((Let "y" i2 y) `Plus` x)
letxx = Let "x" i1 ((Let "x" i2 x) `Plus` x)
noRec = Let "x" x x
noRec' = Let "x" x i1
loop  = Let "x" (Let "x" x x) x
--- function definition
letfx = Let "x" (Let "f" suc (App f i1)) x
--- recursion
fac   = Rec "f" (Fun "x" (IfZ x i1 (x `Mult` (App f (x `Plus` n1)))))
                (App f i3)
fool  = Let "f" (Fun "x" (IfZ x i1 (x `Mult` (App f (x `Plus` n1)))))
                (App f i3)
--- dynamic scope
-- f = Fun "x" (x `Plus` (Fun "y" ()))
dysc = Let "x" i1 
           (Let "f" (Fun "y" (x `Plus` y))
                (Let "x" i2
                     (App f i3)))


-- tracing evaluator
--
evalT :: Stack -> Expr -> Trace Expr Val Val
evalT s a@(N i)        = Tr a s [] (I i)
evalT s a@(Plus e e')  = Tr a s [te,te'] (bop (+) (getVal te) (getVal te'))
                         where (te,te') = (evalT s e,evalT s e')
evalT s a@(Mult e e')  = Tr a s [te,te'] (bop (*) (getVal te) (getVal te'))
                         where (te,te') = (evalT s e,evalT s e')
evalT s a@(IfZ c e e') | isZero (getVal tc) = Tr a s [tc,te]  (getVal te)
                       | otherwise          = Tr a s [tc,te'] (getVal te')
                           where tc  = evalT s c
                                 te  = evalT s e
                                 te' = evalT s e'
evalT s a@(Fun x e)    = Tr a s [] (C x e s) 
evalT s a@(Var x)      = Tr a s [] (getVar x s)
evalT s a@(App f e) | isClos tfv = Tr a s [te,tfe] (getVal tfe) 
                    | otherwise  = Tr a s [] Error
                         where tf       = evalT s f
                               tfv      = getVal tf
                               C x b s' = tfv
                               te    = evalT s e
                               tfe   = evalT ((x,getVal te):s') b
evalT s a@(Let x e e') = Tr a s [te,te'] (getVal te') 
                         where te  = evalT s e
                               te' = evalT ((x,getVal te):s) e'
evalT s a@(Rec x e e') = Tr a s [te,te'] (getVal te') 
                         where te  = evalT s' e
                               te' = evalT ((x,getVal te):s) e'
                               s'  = (x,getVal te):s


tr :: Expr -> Trace Expr Val Val
tr e = evalT [] e

