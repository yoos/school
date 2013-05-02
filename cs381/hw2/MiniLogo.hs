{--- MiniLogo ---}

module MiniLogo where

import SVG

-- Abstract syntax
data MLCmd = Pen Mode
         | MoveTo Int Int
         | Seq MLCmd MLCmd
         deriving Show

data Mode = Up | Down
          deriving (Show, Eq)

type StateML = (Mode, Int, Int)

-- Semantic domain
--type Line = (Int, Int, Int, Int)
--type Lines = [Line]

-- Semantic functions
semS :: MLCmd -> StateML -> (StateML, Lines)
semS (Pen m) (_,x,y) = ((m, x, y), [])
semS (MoveTo i j) (m,x,y) = ((m, i, j), if m==Down then [(x, y, i, j)] else [])
semS (Seq c c') s = (fst (semS c' (fst (semS c s))), snd (semS c s) ++ snd (semS c' (fst (semS c s))))

sem' :: MLCmd -> Lines
sem' c = snd (semS c (Up, 0, 0))

-- Commands for drawing a lambda
yc = Seq (Pen Down) (Seq (MoveTo 1 2) (Seq (Pen Up) (Seq (MoveTo 0 4) (Seq (Pen Down) (MoveTo 2 0)))))

-- Output to SVG.
ysvg = ppLines (sem' yc)

-- vim: expandtab

