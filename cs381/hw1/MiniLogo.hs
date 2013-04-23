module MiniLogo where

data Cmd = Pen Mode
         | MoveTo Pos Pos
         | Def String Pars Cmd
         | Call String Vals
         | Mult Cmd Cmd
         deriving Show

data Mode = Up | Down
          deriving Show

data Pos = PosNum Int
         | PosName String
         deriving Show

data Pars = ParsName String
          | ParsNames String Pars
          deriving Show

data Vals = ValNum Int
          | ValNums Int Vals
          deriving Show



-- def vector (x1, y1, x2, y2) pen up;
--                             moveto (x1, y1);
--                             pen down;
--                             moveto (x2, y2)

vector = Def "vector" (ParsNames "x1" (ParsNames "y1" (ParsNames "x2" (ParsName "y2"))))
                      (Mult (Pen Up)
                      (Mult (MoveTo (PosName "x1") (PosName "y1"))
                      (Mult (Pen Down)
                            (MoveTo (PosName "x2") (PosName "y2")))))

steps :: Int -> Cmd
steps n | n < 1     = (Mult (Pen Up)
                      (Mult (MoveTo (PosNum 0) (PosNum 0))
                            (Pen Down)))
        | otherwise = (Mult (steps (n-1))
                      (Mult (MoveTo (PosNum (n-1)) (PosNum n))
                            (MoveTo (PosNum n) (PosNum n))))

-- vim: expandtab

