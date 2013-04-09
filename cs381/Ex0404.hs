module Ex0404 where

data Grade = A | B | C | D | F
             deriving (Show, Eq, Ord, Enum)

pass :: Grade -> Bool   -- Declare type of function 'pass'
--pass A = True
--pass B = True
--pass C = True
--pass _ = False   -- Catchall
pass g = g < D   -- Evaluates D as an ord.


testPass = map pass [A .. F]   -- Enumerate. TODO: This tests A, B, C, D, and F, but not E. Why?


--type Age = Int
data Age = Year Int
           deriving (Show, Eq, Ord)

birthday :: Age -> Age
birthday (Year y) = Year (y+1)

drinkingAge :: Age
drinkingAge = Year 21

alcoholOK :: Age -> Bool
alcoholOK a = a >= drinkingAge

geq :: Age -> Age -> Bool
geq (Year y) (Year z) = y >= z

jill = Year 20




data Tree = Node Int Tree Tree | Leaf
            deriving (Show, Eq)

t = Node 3 (Node 1 Leaf Leaf)
           (Node 5 Leaf Leaf)

find :: Int -> Tree -> Bool
find _ Leaf = False
--find i (Node i l r) = True   -- This is not allowed because patterns must be linear.
find i (Node j l r) | i==j      = True
                    | i<j       = find i l
                    | otherwise = find i r

--find i (Node j l r) = i==j ||| find i l || find i r   -- Alternative solution


