--
-- Pretty printer for boolean expressions
--

module BoolPP where

import BoolSyn  -- import syntax definition



-- (1) simple version
--
ppBExpr' :: BExpr -> String
ppBExpr' T         = "T"
ppBExpr' F         = "F"
ppBExpr' (Not b)   = "not("++ppBExpr' b++")"
ppBExpr' (Or b b') = "("++ppBExpr' b++") or ("++ppBExpr' b'++")"

-- (2) optimized version, saving parentheses
--
ppBExpr :: BExpr -> String
ppBExpr T         = "T"
ppBExpr F         = "F"
ppBExpr (Not b)   = "not "++ppBExprParen b
ppBExpr (Or b b') = ppBExprParen b++" or "++ppBExprParen b'

ppBExprParen :: BExpr -> String
ppBExprParen b@(Not _)  = paren (ppBExpr b)
ppBExprParen b@(Or _ _) = paren (ppBExpr b)
ppBExprParen b          = ppBExpr b

paren :: String -> String
paren s = "("++s++")"


-- For use with GHC
-- 
instance Show BExpr where
  show = ppBExpr

