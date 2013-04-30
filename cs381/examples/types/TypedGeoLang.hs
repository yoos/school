--
-- Typed geometric language
--
module TypedGeoLang where


-- Basic geometric types
--
type Point = (Int,Int)

type Line = (Point,Point)

type Polygon = [Point]

type Vector = Point


-- Language for constructing geometric objects
--
data Geo =
           -- Operations for constructing basic objects
           Pt Point
         | Ln Line
         | Poly Polygon
           -- Operations for combining and comparing objects
         | Union Geo Geo
         | Intersection Geo Geo
           -- Operations for modifying objects
         | Translate Vector Geo
         | Scale Int Geo


-- A data type for representing the types of the 
-- geometric language
--
data Type = Points | Lines | Region | TypeError
            deriving Eq


-- A static type checker
--
tc :: Geo -> Type
tc (Pt _)   = Points
tc (Ln _)   = Lines
tc (Poly _) = Region
tc (Union a b) | tc a == tc b = tc a
               | otherwise    = TypeError
tc (Intersection a b) =
      case (tc a,tc b) of
         (TypeError,_)     -> TypeError
         (_,TypeError)     -> TypeError
         (Points,_)        -> Points
         (_,Points)        -> Points
         (Lines,Lines)     -> Points  -- approximation!
         (Lines,Region)    -> Lines
         (Region,Lines)    -> Lines
         (Region,Region)   -> Region
tc (Translate v g) = tc g


