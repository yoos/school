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

type Macros = [(String, Prog)]   -- List of macro names and the programs they represent.
type State = (Macros, Stack)   -- Language state.

-- Semantic function
sem2 :: Prog -> D
sem2 [] i = i
sem2 (o:os) i = sem os (semCmd2 o i)

semCmd2 :: ExtCmd -> D
semCmd2 (LD i) is =   -- Load i onto stack.
    case is of
        Just is -> Just (is ++ [i])
        _       -> Nothing
semCmd2 ADD is =   -- Remove two topmost integers from stack and put their sum onto stack.
    case is of
        Just is -> if length is < 2 then Nothing
                                    else Just (init(init is) ++ [last is + last(init is)])
        _       -> Nothing
semCmd2 MULT is =   -- Remove two topmost integers from stack and put their product onto stack.
    case is of
        Just is -> if length is < 2 then Nothing
                                    else Just (init(init is) ++ [last is * last(init is)])
        _       -> Nothing
semCmd2 DUP is =   -- Place second copy of topmost integer onto stack.
    case is of
        Just is -> if length is < 1 then Nothing
                                    else Just (is ++ [last is])
        _       -> Nothing




-- vim: expandtab

