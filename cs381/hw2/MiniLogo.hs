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

type StateML = (Mode, Int, Int)

-- Semantic domain
--type Line = (Int, Int, Int, Int)
--type Lines = [Line]

-- Semantic functions
semS :: Cmd -> StateML -> (StateML, Lines)


sem' :: Cmd -> Lines
--TODO


sml :: StateML
sml = (Up, 0, 0)

-- vim: expandtab

