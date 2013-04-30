{--- StackLang ---}

module StackLang where

type Prog = [Cmd]
type Stack = [Int]

-- Abstract syntax
data Cmd = LD Int
         | ADD
         | MULT
         | DUP
         deriving Show

-- Semantic domain defined as Maybe to account for errors.
type D = Maybe Stack -> Maybe Stack

-- Semantic function
sem :: Prog -> D
sem [] i = i
sem (o:os) i = sem os (semCmd o i)

semCmd :: Cmd -> D
semCmd (LD i) is =   -- Load i onto stack.
    case is of
        Just is -> Just (is ++ [i])
        _       -> Nothing
semCmd ADD is =   -- Remove two topmost integers from stack and put their sum onto stack.
    case is of
        Just is -> if length is < 2 then Nothing
                                    else Just (init(init is) ++ [last is + last(init is)])
        _       -> Nothing
semCmd MULT is =   -- Remove two topmost integers from stack and put their product onto stack.
    case is of
        Just is -> if length is < 2 then Nothing
                                    else Just (init(init is) ++ [last is * last(init is)])
        _       -> Nothing
semCmd DUP is =   -- Place second copy of topmost integer onto stack.
    case is of
        Just is -> if length is < 1 then Nothing
                                    else Just (is ++ [last is])
        _       -> Nothing

-- Some programs
p = [LD 3, DUP, ADD, DUP, MULT]
p' = [LD 3, ADD]
p'' = []

-- A stack
s :: Maybe [Int]   -- Without this, GHC assumes s is of type Maybe [Integer] instead of Maybe [Int]. Why?
s = Just [1, 2, 3]


-- vim: expandtab

