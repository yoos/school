module STrace (Trace(..),getState,concatT,substStmt,showSt) where

import Data.List (intersperse)


type St a = [(String,a)]

data Trace s a = Tr [(s,St a)]

instance (Show s,Show a) => Show (Trace s a) where
  show  = showT

showT :: (Show s,Show a) => Trace s a -> String
showT (Tr t) = concatMap (\(s,m)->show s++"    "++showSt m++"\n") t


getState :: Trace s a -> (St a)
getState (Tr t) = snd (last t)

concatT :: Trace s a -> Trace s a -> Trace s a
concatT (Tr t) (Tr t') = Tr (t++t')

substStmt :: s -> Trace s a -> Trace s a
substStmt s (Tr ((_,m):ts)) = Tr ((s,m):ts)

showSt :: Show a => St a -> String
showSt s = '[':concat (intersperse "," (map showPair s))++"]"
                    where showPair (n,v) = n++":"++show v


