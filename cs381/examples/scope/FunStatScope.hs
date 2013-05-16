-- 
-- Extending arithmetic expressions by functions (static scoping)
--
module FunStatScope where

import Trace 


type Name = String

data Expr = N Int                -- integer constant
          | Plus Expr Expr       -- addition
          | Var Name             -- reference to a variable
          | Let Name Expr Expr   -- local definition
          | Fun Name Expr        -- function abstraction
          | App Expr Expr        -- application

data Val = I Int              -- integer constants
         | C Name Expr Stack  -- closures (stack remembers values from definition)
         | Error

showFun x e = "\\"++x++"->"++show e
                  

instance Show Expr where
  show (N i)        = show i
  show (Plus e e')  = showP e++"+"++showP e'
  show (Var x)      = x
  show (Let x e e') = "let "++x++"="++show e++" in "++show e'
  show (Fun x e)    = showFun x e
  show (App e e')   = case e of 
                        App _ _ -> show e++" "++showP e'
                        _       -> showP e++" "++showP e'

showClos x e s = showFun x e++";@"++show (length s)

showP e@(N _)   = show e
showP e@(Var _) = show e
showP e         ="("++show e++")"

instance Show Val where
  show (I i)     = show i
  show (C x e s) = showClos x e s
  show Error     = "error"


type Stack = [(Name,Val)]


-- The semantics is defined as a function that maps
-- expressions of type Expr to values. Values are
-- integers, functions, or errors. Note that eval
-- takes as an additional argument a runtime stack
-- for storing local definitions of variables.
--
eval :: Stack -> Expr -> Val
eval _ (N i)        = I i
eval s (Plus e e')  = add (eval s e) (eval s e')
eval s (Var x)      = getVar x s
eval s (Let x e e') = eval ((x,eval s e):s) e'
eval s (Fun x e)    = C x e s 
eval s (App f e')   = case eval s f of
                        C x e s' -> eval ((x,eval s e'):s') e
                        _        -> Error


add :: Val -> Val -> Val
add (I i) (I j) = I (i+j)
add _     _     = Error

getVar :: Name -> Stack -> Val
getVar x s = case lookup x s of
               Just v  -> v
               Nothing -> Error


-- "smart constructors" to simplify the construction
-- of syntax trees
-- 
[i0,i1,i2,i3] = map N [0 .. 3] 
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
--- CBValue vs. CBName evaluation
one = Let "x" y i1
four = Let "f" suc (App f (i1 `Plus` i2))
--- dynamic scope
-- f = Fun "x" (x `Plus` (Fun "y" ()))
dysc = Let "x" i1 
           (Let "f" (Fun "y" (y `Plus` x))
                (Let "x" i2
                     (App f i0)))
d2 = Let "z" i1 dysc
d3 = Let "x" i1 
           (Let "g" (Fun "x" (x `Plus` x))
                (Let "x" i3
                     d2))


-- tracing evaluator
--
isClos :: Val -> Bool
isClos (C _ _ _) = True
isClos _         = False

evalT :: Stack -> Expr -> Trace Expr Val Val
evalT s a@(N i)        = Tr a s [] (I i)
evalT s a@(Plus e e')  = Tr a s [te,te'] (add (getVal te) (getVal te'))
                         where (te,te') = (evalT s e,evalT s e')
evalT s a@(Var x)      = Tr a s [] (getVar x s)
evalT s a@(Let x e e') = Tr a s [te,te'] (getVal te') 
                         where te  = evalT s e
                               te' = evalT ((x,getVal te):s) e'
evalT s a@(Fun x e)    = Tr a s [] (C x e s) 
evalT s a@(App f e) | isClos tfv = Tr a s [te,tfe] (getVal tfe) 
                    | otherwise  = Tr a s [] Error
                         where tf       = evalT s f
                               tfv      = getVal tf
                               C x b s' = tfv
                               te    = evalT s e
                               tfe   = evalT ((x,getVal te):s') b

tr :: Expr -> Trace Expr Val Val
tr e = evalT [] e

