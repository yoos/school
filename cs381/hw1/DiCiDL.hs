{--- Digital Circuit Design Language ---}

module DiCiDL where

{- 2a: Abstract syntax definition -}

data Circuit = Circ Gates Links
--             deriving Show

data Gates = Gate Int GateFn Gates
           | GatesE
--           deriving Show

data GateFn = AND | OR | XOR | NOT
--            deriving Show

data Links = Link (Int, Int) (Int, Int) Links
           | LinksE
--           deriving Show


{- 2b: Half adder -}

halfAdder = Circ (Gate 1 XOR
                 (Gate 2 AND GatesE))
                 (Link (1, 1) (2, 1)
                 (Link (1, 2) (2, 2) LinksE))


{- 2c: Pretty printer -}

instance Show Circuit where
  show (Circ gs ls) = show gs ++ show ls

instance Show Gates where
  show (Gate n gf gs) = show n  ++ ": "  ++
                        show gf ++ ";\n" ++
                        show gs
  show GatesE = ""

instance Show GateFn where
  show AND = "AND"
  show OR  = "OR"
  show XOR = "XOR"
  show NOT = "NOT"

instance Show Links where
  show (Link (a, b) (c, d) ls) = "from " ++ show a ++ "." ++ show b ++
                                 " to "  ++ show c ++ "." ++ show d ++
                                 ";\n"   ++ show ls
  show LinksE = ""


-- vim: expandtab

