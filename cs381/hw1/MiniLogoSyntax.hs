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

data Pars = Name1 String        -- 1 name
          | NameN String Pars   -- n names
          deriving Show

data Vals = Num1 Int            -- 1 number
          | NumN Int Vals       -- n numbers
          deriving Show


-- vim: expandtab

