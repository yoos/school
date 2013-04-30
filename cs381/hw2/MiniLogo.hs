{--- MiniLogo ---}

module MiniLogo where

import SVG

-- Abstract syntax
data Cmd = Pen Mode
         | MoveTo Int Int
         | Seq Cmd Cmd
         deriving Show

data Mode = Up | Down
          deriving Show

type State = (Mode, Int, Int)

-- Semantic domain
--type Line = (Int, Int, Int, Int)
--type Lines = [Line]

-- Semantic functions
semS :: Cmd -> State -> (State, Lines)


sem' :: Cmd -> Lines
--TODO


s :: State
s = (Up, 0, 0)

-- vim: expandtab

