--
-- Boolean expressions
--

module BoolSyn where


-- Boolean expressions
--
data BExpr = T | F | Not BExpr
           | Or BExpr BExpr
--              deriving Show  -- uncomment for GHC
--
-- data Bool = True | False   -- is predefined

nnt :: BExpr
nnt = Not (Not T)

tonnt :: BExpr
tonnt = Or T nnt

