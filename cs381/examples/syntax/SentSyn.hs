--
-- Sentence syntax
--

module SentSyn where


-- For GHC we need "deriving Show" to print data type values
-- See also file SentPP.hs 
--
data Sentence = Phrase Noun Verb Noun 
              | And Sentence Sentence 
              deriving Show

--instance Show Sentence where
--   show (Phrase s v o) = show s++" "++show v++" "++show o
--   show (And s1 s2) = show s1++" and "++show s2

data Noun = Dogs | Teeth  deriving Show
data Verb = Have          deriving Show


-- example sentences
--
s1 :: Sentence
s1  = Phrase Dogs Have Teeth

s2 :: Sentence
s2 = Phrase Teeth Have Dogs

s3 :: Sentence
s3 = And s1 s2


-- Two error examples (the second is not an error in GHC
-- because it represents a 'partial sentence', that is, a
-- function that when applied to a noun yields a proper 
-- sentence)
--
--err1 = Phrase Dogs Have Have
err2 = Phrase Dogs Have












