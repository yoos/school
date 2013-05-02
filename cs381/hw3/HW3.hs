module HW3 where

{- Exercise 1: Rank-based type system for the stack language -}

type Prog = [Cmd]

data Cmd = LD Int
         | ADD
         | MULT
         | DUP
         | INC
         | SWAP
         | POP Int
         deriving Show


-- vim: expandtab

