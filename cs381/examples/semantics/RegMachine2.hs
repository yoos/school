--
-- Simple machine with one register
--
module RegMachine where

-- syntax of the assembly language
-- 
type Prog = [Op]

data Reg = A | B deriving Show

data Op = LD Reg Int
        | INC Reg
        | DUP Reg
          deriving Show

-- semantics of an operation and a program is a 
-- transformation of the register content.
--
type RegContent = (Int,Int)

type D = RegContent -> RegContent

-- semOp :: Op -> D
-- semOp :: Op -> RegContent -> RegContent
-- semOp (LD A i) (a,b) = (i,b)
-- semOp (INC A)  (a,b) = (a+1,b)
-- semOp (DUP A)  (a,b) = (a*2,b)
-- semOp (LD B i) (a,b) = (a,i)
-- semOp (INC B)  (a,b) = (a,b+1)
-- semOp (DUP B)  (a,b) = (a,b*2)

onReg :: Reg -> (Int -> Int) -> RegContent -> RegContent
onReg A f (a,b) = (f a,b)
onReg B f (a,b) = (a,f b)

semOp :: Op -> RegContent -> RegContent
-- semOp (LD r i) = onReg r (\_->i)
semOp (LD r i) = onReg r (const i)
semOp (INC r)  = onReg r (+1)
semOp (DUP r)  = onReg r (*2)

-- sem :: Prog -> D
sem :: Prog -> RegContent -> RegContent
sem []     m = m
sem (o:os) m = sem os (semOp o m)

-- machine interpreter "starts with" 0 in both registers
--
eval :: Prog -> RegContent
eval p = sem p (0,0)

-- example program
--
p :: Prog 
p = [LD A 2,INC B,DUP A]

