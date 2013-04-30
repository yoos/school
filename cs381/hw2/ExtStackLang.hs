{--- ExtStackLang ---}

module ExtStackLang where

type ExtProg = [ExtCmd]
type Stack = [Int]

-- Abstract syntax
data ExtCmd = EXTLD Int
            | EXTADD
            | EXTMULT
            | EXTDUP
            | DEF String
            | CALL String
            deriving Show

-- Semantic domain defined as Maybe to account for errors.
type D = Maybe State -> Maybe State

type Macros = [(String, ExtProg)]   -- List of macro names and the programs they represent.
type State = (Macros, Stack)   -- Language state.

-- Semantic function
sem2 :: ExtProg -> D
sem2 [] i = i
sem2 (o:os) i = sem2 os (semCmd2 o i)

semCmd2 :: ExtCmd -> D
semCmd2 (EXTLD i) (Just (ms,is)) =   -- Load i onto stack.
    case Just is of
        Just is -> Just (ms, is ++ [i])
        _       -> Nothing
semCmd2 EXTADD (Just (ms,is)) =   -- Remove two topmost integers from stack and put their sum onto stack.
    case Just is of
        Just is -> if length is < 2 then Nothing
                                    else Just (ms, init(init is) ++ [last is + last(init is)])
        _       -> Nothing
semCmd2 EXTMULT (Just (ms,is)) =   -- Remove two topmost integers from stack and put their product onto stack.
    case Just is of
        Just is -> if length is < 2 then Nothing
                                    else Just (ms, init(init is) ++ [last is * last(init is)])
        _       -> Nothing
semCmd2 EXTDUP (Just (ms,is)) =   -- Place second copy of topmost integer onto stack.
    case Just is of
        Just is -> if length is < 1 then Nothing
                                    else Just (ms, is ++ [last is])
        _       -> Nothing

{-
 - In the above semantic function definition, I wanted to do something like
 - this:
 -
 -     semCmd2 (EXTLD i) (ms,is) =
 -         case is of
 -             Just is -> Just (ms, is ++ [i])
 -             _       -> Nothing
 -
 - but GHC complained that it "Couldn't match expected type `Maybe State' with
 - actual type `(t0, t1)'"
 -
 - What I've done instead seems like an unnecessary hack.
 -}



-- vim: expandtab

