{--- ExtStackLang ---}

module ExtStackLang where

type Prog = [ExtCmd]
type Stack = [Int]

-- Abstract syntax
data ExtCmd = LD Int
            | ADD
            | MULT
            | DUP
            | DEF String
            | CALL String
            deriving Show

-- Semantic domain defined as Maybe to account for errors.
type D = Maybe Stack -> Maybe Stack

-- vim: expandtab

