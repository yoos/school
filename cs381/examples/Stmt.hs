module Stmt where

-- Stmt syntax
--
data Cond = T 
          | Not Cond 
          deriving Show

data Stmt = While Cond Stmt 
          | Noop
          deriving Show


ppCond :: Cond -> String
ppCond T       = "T"
ppCond (Not c) = "not("++ppCond c++")"

-- simple version: no indentation
--
ppStmt :: Stmt -> String
ppStmt Noop        = "noop"
ppStmt (While c s) = "while "++ppCond c++" {\n"++
                     ppStmt s++"\n}"

-- improved version with indentation
-- take and additional integer parameter indicating indentation level
--
indent :: Int -> String
indent i = take (4*i) (repeat ' ')

ppStmtI :: Int -> Stmt -> String
ppStmtI i Noop        = indent i++"noop"
ppStmtI i (While c s) = indent i++"while "++ppCond c++" {\n" ++
                        ppStmtI (i+1) s ++ "\n" ++
                        indent i++"}"


-- pretty printer produces string and sends it
-- to the standard output
--
pp :: Stmt -> IO ()
--pp s = putStrLn (ppStmt s)  -- without indentation
pp s = putStrLn (ppStmtI 0 s)  -- with indentation


-- examples
--
p1 :: Stmt
p1 = While T Noop

p2 :: Stmt
p2 = While (Not (Not T)) p1

p3 :: Stmt
p3 = While (Not T) (While (Not T) p2)



-- syntax transformations
-- 
simplifyC :: Cond -> Cond
simplifyC (Not (Not c)) = simplifyC c
simplifyC c             = c

simplifyS :: Stmt -> Stmt
simplifyS (While c Noop)    = Noop
simplifyS (While (Not T) s) = Noop

simplify :: Stmt -> Stmt
simplify (While c s) = simplifyS (While (simplifyC c) (simplify s)) 
simplify s = s

