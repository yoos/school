--
-- Semantics of boolean expressions
--

module BoolSem where

-- import BoolSyn  -- import syntax definition
-- import BoolPP

data BExpr = T | F
           | Not BExpr
           | Or BExpr BExpr
           deriving Show 

sem :: BExpr -> Bool
sem T         = True
sem F         = False
sem (Not b)   = not (sem b)
sem (Or b b') = sem b || sem b'
-- sem (Or b b') | sem b     = True
--               | otherwise = sem b'

fonnf = F `Or` Not (Not F)


f = fonnf `Or` fonnf
t = T `Or` fonnf
