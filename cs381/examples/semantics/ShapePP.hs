-- 
-- Semantics of shape language with pretty printing of images
--
module Shape where


import Data.List (sortBy,groupBy)


-- (1) Syntax of shapes
--
data Shape = X 
           | TD Shape Shape
           | LR Shape Shape
           deriving Show


-- (2) Semantic domain: an image is a set of pixels
--
type Pixel = (Int,Int)
-- 
-- make image a data type to allow making it
-- an instance of Show 
--
data Image = I [Pixel]

instance Show Image where
  show (I xs) = show xs++"\n\n"++ppImage xs

--
-- printing images
--
ppImage :: [Pixel] -> String
ppImage xs = unlines (map (prnLine 1) lines)
             where lines = reverse . groupOn snd . sortBy cmpPixel $ xs

ppI = groupOn snd . sortBy cmpPixel 

sortOn f = sortBy (\x y->compare (f x) (f y))
groupOn f = groupBy (\x y->f x==f y)

cmpPixel :: Pixel -> Pixel -> Ordering
cmpPixel (a,b) (c,d) = if b<d || (b==d && a<c) then LT else
                       if b>d || (b==d && a>c) then GT else EQ

prnLine :: Int -> [Pixel] -> String
prnLine _ []         = ""
prnLine i ((x,y):xs) | i==x      = 'X':prnLine (i+1) xs
                     | otherwise = ' ':prnLine (i+1) ((x,y):xs)

-- (3) Semantic function
--
sem :: Shape -> Image 
sem X          = I [(1,1)]
sem (LR s1 s2) = I (d1 ++ [(x+maxx d1,y) | (x,y) <- sem' s2]) where I d1 = sem s1
sem (TD s1 s2) = I (d2 ++ [(x,y+maxy d2) | (x,y) <- sem' s1]) where I d2 = sem s2

sem' s = xs where I xs = sem s

maxx :: [Pixel] -> Int
maxx = maximum . map fst

maxy :: [Pixel] -> Int
maxy = maximum . map snd


-- examples
-- 
s1 = LR (TD X X) X
p1 = sem s1

s2 = TD X (LR X X)
p2 = sem s2

s3 = TD (LR X X) X
p3 = sem s3

hor 1 = X
hor n = LR X (hor (n-1))

ver 1 = X
ver n = TD X (ver (n-1))

corner = LR (hor 3) (ver 2)
cup = LR (ver 2) (LR (hor 2) (ver 2))
cap = LR (TD (hor 3) X) (ver 2)

three = TD (hor 4) (TD corner corner)
eight = TD cap (TD cap (hor 4))
one = ver 5

dot s = LR s (hor 3)

teo = LR (dot three) (LR (dot eight) one)

