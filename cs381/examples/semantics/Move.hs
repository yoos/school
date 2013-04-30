-- 
-- Semantics of move language
-- 
module Move where


-- (1) Syntax of move language
--
data Dir  = Lft | Rgt | Up | Dwn  
            deriving Show
            -- note: Left and Right are already defined

data Step = Go Dir Int
            deriving Show

type Move = [Step]


-- (2) Semantic domain
--
type Pos = (Int,Int)


-- (3) Semantic function
--
sem :: Move -> Pos
sem []  = (0,0)
sem (Go d i:ss) = (dx*i+x,dy*i+y) 
                  where (x,y)   = sem ss
                        (dx,dy) = vector d

vector :: Dir -> (Int,Int)
vector Lft = (-1,0)
vector Rgt = (1,0)
vector Up  = (0,1)
vector Dwn = (0,-1)


-- examples
-- 
m :: Move
m = [Go Up 3,Go Rgt 4,Go Dwn 1]

