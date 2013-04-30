{--- MiniLogo ---}

module MiniLogo where

import SVG

-- Abstract syntax
data Cmd = Pen Mode
         | MoveTo Int Int
         | Seq Cmd Cmd
         deriving Show

data Mode = Up | Down
          deriving (Show, Eq)

type StateML = (Mode, Int, Int)

-- Semantic domain
--type Line = (Int, Int, Int, Int)
--type Lines = [Line]

-- Semantic functions
semS :: Cmd -> StateML -> (StateML, Lines)
semS (Pen m) (_,x,y) = ((m, x, y), [])
semS (MoveTo i j) (m,x,y) = ((m, i, j), if m==Down then [(x, y, i, j)] else [])
semS (Seq c c') s = (fst (semS c' (fst (semS c s))), snd (semS c s) ++ snd (semS c' (fst (semS c s))))

sem' :: Cmd -> Lines
sem' c = snd (semS c (Up, 0, 0))


-- vim: expandtab

