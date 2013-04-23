module MiniLogoSyntax where

data Cmd = Pen Mode
         | MoveTo Pos Pos
         | Def String Pars Cmd
         | Call String Vals
         | Mult Cmd Cmd
         deriving Show

data Mode = Up | Down
          deriving Show

data Pos = Num Int
         | Name String
         deriving Show

data Pars = Name String
          | Name String Pars
          deriving Show

data Vals = Num Int
          | Num Int Vals
          deriving Show


-- vim: expandtab

