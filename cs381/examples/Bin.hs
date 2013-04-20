module Bin where

data Digit = One | Zero
             deriving Show

data Bin = D Digit
         | B Digit Bin
         deriving Show

l0l = B One (B Zero (D One))

