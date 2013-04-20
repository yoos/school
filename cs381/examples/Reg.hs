--
-- Simple machine with two registers
--
module Reg where

{-

con	::=  0 | 1 
reg	::=  A | B | C 
op	::=  MOV con TO reg 
     |   MOV reg TO reg 
     |   INC reg BY con 
     |   INC reg BY reg

-}

-- data Con = Zero | One
type Con = Int

data Reg = A | B | C deriving (Eq,Show)

data Op = MOVC Con Reg
        | MOVR Reg Reg
        | INCC Reg Con
        | INCR Reg Reg

-- refactored representation
--
data ConOrReg = X Con | R Reg deriving (Eq, Show)

data Op' = MOV ConOrReg Reg
         | INC Reg      ConOrReg
         deriving Show


type Prog = [Op']


-- Optimizer
--
simplify :: Prog -> Prog
simplify (MOV _ r:MOV a s:p) 
       | r==s  = simplify (MOV a r:p)

simplify (INC r (X i):INC s (X j):p) 
       | r==s  = simplify (INC r (X (i+j)):p)

simplify (c:p) = c:simplify p
simplify p = p

p = [MOV (X 3) A,MOV (X 5) B,MOV (X 7) B,
     INC B (X 2),INC B (X 11)]






