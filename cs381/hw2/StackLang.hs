{--- StackLang ---}

module StackLang where

type Prog = [Cmd]

-- Abstract syntax
data Cmd = LD Int
         | ADD
         | MULT
         | DUP
         deriving Show

type Stack = [Int]

type D = Stack -> Stack


sem :: Prog -> D   -- Semantic domain
sem [] i = i
sem (o:os) i = sem os (semCmd o i)

semCmd :: Cmd -> D
semCmd LD     = \_ -> i
semCmd ADD    = \i j -> i + j
semCmd MULT   = \i j -> i * j
semCmd DUP    = \i -> i * 2


-- vim: expandtab

