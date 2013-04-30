{--- ExtStackLang ---}

module ExtStackLang where

type ExtProg = [ExtCmd]
type Stack = [Int]

-- Abstract syntax
data ExtCmd = EXTLD Int
            | EXTADD
            | EXTMULT
            | EXTDUP
            | DEF String [ExtCmd]
            | CALL String
            deriving Show

-- Semantic domain defined as Maybe to account for errors.
type EXTD = Maybe State -> Maybe State

type Macros = [(String, ExtProg)]   -- List of macro names and the programs they represent.
type State = (Macros, Maybe Stack)   -- Language state.

-- Semantic function with plenty of error checking. Does this really have to be
-- so messy?
sem2 :: ExtProg -> EXTD
sem2 [] i = i
sem2 (o:os) i = sem2 os (semCmd2 o i)

semCmd2 :: ExtCmd -> EXTD
semCmd2 (EXTLD i) (Just (ms, Just is)) =   -- Load i onto stack.
    case Just is of
        Just is -> Just (ms, Just (is ++ [i]))
        _       -> Nothing
semCmd2 EXTADD (Just (ms, Just is)) =   -- Remove two topmost integers from stack and put their sum onto stack.
    case Just is of
        Just is -> if length is < 2 then Nothing
                                    else Just (ms, Just (init(init is) ++ [last is + last(init is)]))
        _       -> Nothing
semCmd2 EXTMULT (Just (ms, Just is)) =   -- Remove two topmost integers from stack and put their product onto stack.
    case Just is of
        Just is -> if length is < 2 then Nothing
                                    else Just (ms, Just (init(init is) ++ [last is * last(init is)]))
        _       -> Nothing
semCmd2 EXTDUP (Just (ms, Just is)) =   -- Place second copy of topmost integer onto stack.
    case Just is of
        Just is -> if length is < 1 then Nothing
                                    else Just (ms, Just (is ++ [last is]))
        _       -> Nothing
semCmd2 (DEF s cs) (Just (ms, Just is)) =   -- Define a macro.
    case Just is of
        Just is -> Just (ms ++ [(s, cs)], Just is)
        _       -> Nothing
semCmd2 (CALL s) (Just (ms, Just is)) =   -- Call a macro.
    case (ms, Just is) of
        (ms, Just is) -> if (isMacro s ms) then sem2 (getMacro s ms) (Just (ms, Just is))
                                          else Nothing
        _       -> Nothing

-- Helper functions for finding macro program s in list of macros ms.
isMacro :: String -> Macros -> Bool
isMacro s ms = if length (filter ((==s).fst) ms) > 0 then True else False

getMacro :: String -> Macros -> ExtProg
getMacro s ms = snd (head (filter ((==s).fst) ms))

-- Some programs
ep = [EXTLD 3, EXTDUP, EXTADD, EXTDUP, EXTMULT]
ep' = [EXTLD 3, EXTADD]
ep'' = []
ep''' = [EXTADD, DEF "a" ep', CALL "a"]

-- Some stacks
es :: Maybe Stack
es = Just [1, 2, 3, 4]
es' = Just [1]

-- Language state
ls :: Maybe State
ls = Just ([], es)



-- vim: expandtab

