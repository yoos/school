module Trace (Trace(..),getVal,getExpr) where

import Data.List (intersperse)


type Stk a = [(String,a)]

data Trace e a v = Tr e (Stk a) [Trace e a v] v 

instance (Show e,Show a,Show v) => Show (Trace e a v) where
  show  = showInd 0

showInd :: (Show e,Show a,Show v) => Int -> Trace e a v -> String
showInd i (Tr e s ts v) = indent i ">>"++show e++"    "++showStk s++"\n"++
                          concatMap (showInd (i+1)) ts++
                          indent i " ="++show v++"\n"

indent :: Int -> String -> String
indent i s = take (4*i) (repeat ' ')++s++" "

getVal :: Trace e a v -> v
getVal (Tr _ _ _ v) = v

getExpr :: Trace e a v -> e
getExpr (Tr e _ _ _) = e

showStk :: Show a => Stk a -> String
showStk s = '[':concat (intersperse "," (map showPair s))++"]"
                    where showPair (n,v) = n++":"++show v


