module DiCiDL where

data Circuit = Circ Gates Links
             deriving Show

data Gates = Gate Int GateFn Gates
           | GatesE
           deriving Show

data GateFn = AND | OR | XOR | NOT
            deriving Show

data Links = Link (Int, Int) (Int, Int) Links
           | LinksE
           deriving Show


halfAdder = Circ (Gate 1 XOR
                 (Gate 2 AND GatesE))
                 (Link (1, 1) (2, 1)
                 (Link (1, 2) (2, 2) LinksE))

-- vim: expandtab

