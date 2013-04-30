{--- StackLang ---}

module StackLang where

type Prog = [Cmd]

data Cmd = LD Int
         | ADD
         | MULT
         | DUP
         deriving Show

type Stack = [Int]



-- vim: expandtab

