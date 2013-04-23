module DiCiDL where

data Circuit = Mult Gates Links
             deriving Show

data Gates = GateNum Int GateFn Gates
           | GatesE
           deriving Show

data GateFn = AND | OR | XOR | NOT
            deriving Show

data Links = Link (Int, Int) (Int, Int) Links
           | LinksE
           deriving Show


-- vim: expandtab

