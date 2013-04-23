module HW1 where

{--- MiniLogo ---}

{- 1a: Abstract syntax definition -}

data Cmd = Pen Mode
         | MoveTo (Pos, Pos)
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


{- 1b: vector macro -}

-- def vector (x1, y1, x2, y2) pen up;
--                             moveto (x1, y1);
--                             pen down;
--                             moveto (x2, y2)

vector = Def "vector" (ParsNames "x1" (ParsNames "y1" (ParsNames "x2" (ParsName "y2"))))
                      (Mult (Pen Up)
                      (Mult (MoveTo (PosName "x1", PosName "y1"))
                      (Mult (Pen Down)
                            (MoveTo (PosName "x2", PosName "y2")))))


{- 1c: Step-drawing function -}

steps :: Int -> Cmd
steps n | n < 1     = (Mult (Pen Up)
                      (Mult (MoveTo (PosNum 0, PosNum 0))
                            (Pen Down)))
        | otherwise = (Mult (steps (n-1))
                      (Mult (MoveTo (PosNum (n-1), PosNum n))
                            (MoveTo (PosNum n, PosNum n))))


{--- Digital Circuit Design Language ---}

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


{--- Designing Abstract Syntax ---}

data Expr = N Int
          | Plus Expr Expr
          | Times Expr Expr
          | Neg Expr
          deriving Show

data Op = Add | Multiply | Negate
        deriving Show

data Exp = Num Int
         | Apply Op [Exp]
         deriving Show


origExp = Times (Neg (Plus (N 3) (N 4))) (N 7)


{- 3a: Alternative expression -}

altExp = Apply Multiply [Apply Negate [Apply Add [Num 3, Num 4]], Num 7]


{- 3b: Advantages/disadvantages -}

-- The alternate expression has no contraint on how many operands an operator
-- can be applied to, which makes it more concise when there are more than two
-- operands. On the flipside, because there is no constraint, it is possible to
-- apply an operator such as Add to only a single or no operand, which makes no
-- sense. The flexibility of the alternative representation allows the user to
-- more freely express expressions but also leaves it up to the user to do so
-- properly.


{- 3c: Translator -}

translate :: Expr -> Exp
translate (N a) = Num a
translate (Plus a b) = Apply Add [translate a, translate b]
translate (Times a b) = Apply Multiply [translate a, translate b]
translate (Neg a) = Apply Negate [translate a]


-- vim: expandtab

