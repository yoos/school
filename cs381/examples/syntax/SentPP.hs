--
-- Pretty printer for sentence syntax
--

module SentPP where

import SentSyn  -- import syntax definition



-- Implicit pretty printing via show
-- 
instance Show Noun where
  show Dogs  = "dogs"
  show Teeth = "teeth"

instance Show Verb where
  show Have = "have"

instance Show Sentence where
  show (Phrase n v n') = show n++" "++show v++" "++show n'
  show (And s s')      = show s++" and "++show s'



-- Explicit pretty printing
--
ppNoun :: Noun -> String
ppNoun Dogs  = "dogs"
ppNoun Teeth = "teeth"

ppVerb :: Verb -> String
ppVerb Have = "have"

ppSent :: Sentence -> String
ppSent (Phrase n v n') = ppNoun n++" "++ppVerb v++" "++ppNoun n'
ppSent (And s s')      = ppSent s++" and\n "++ppSent s'


