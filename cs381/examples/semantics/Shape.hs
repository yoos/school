-- 
-- Semantics of shape language
--
module Shape where


-- (1) Syntax of shapes
--
data Shape = X 
           | TD Shape Shape
           | LR Shape Shape
           deriving Show


-- (2) Semantic domain: an image is a set of pixels
--
type Pixel = (Int,Int)
type Image = [Pixel]


-- (3) Semantic function
--
sem :: Shape -> Image 
sem X           = [(1,1)]
sem (LR s1 s2) = d1 ++ [(x+maxx d1,y) | (x,y) <- sem s2] 
                 where d1 = sem s1
sem (TD s1 s2) = d2 ++ [(x,y+maxy d2) | (x,y) <- sem s1] 
                 where d2 = sem s2

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


-- adding concrete syntax (if one feels like it)
--
pixel = X

onTopOf = TD
leftOf = LR
place = id

c1 = place (place pixel `onTopOf` pixel) `leftOf` pixel

c1' = place rectangle `leftOf` pixel
      where rectangle = place pixel `onTopOf` pixel
