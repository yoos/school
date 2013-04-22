module MiniLogoSyntax where

data Cmd = Pen Mode
         | MoveTo Pos Pos
         | Def Name Pars Cmd
         | Call Name Vals
         | Cmd Cmd



-- vim: expandtab

