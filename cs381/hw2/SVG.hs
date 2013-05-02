module SVG where
    
-- 
-- The definitions from this file can be used to
-- render the semantics of Mini Logo.
--
-- NOTE: you have to put the following import
-- statement into the Haskell module to use the
-- function ppLines.
--

import System.IO

-- 
-- NOTE: The import statement must be the first 
-- statement in the file (after the module header).
--


-- defintion of semantic domains
-- 
type Line  = (Int,Int,Int,Int)
type Lines = [Line]


-- Pretty printing of lines:
-- write an svg file
--
ppLines :: Lines -> IO ()
ppLines ls = do h <- openFile "MiniLogo.svg" WriteMode
                hPutStr h (svgHdr++concatMap ppLine ls++svgFtr)
                hClose h

-- fixed size and maginifaction factor
-- (can be generalized easily)
--
factor=100
yMax=1100

svgHdr = "<?xml version=\"1.0\" standalone=\"no\"?>\n \
         \ <!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n \
         \    \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n \
         \ <svg width=\"12cm\" height=\"11cm\" viewBox=\"0 0 1200 1100\"\n \
         \    xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\">\n \
         \ <title>Mini Logo Result Viewer</title>\n \
         \ <desc>A set of line segments</desc>\n \
         \ <rect x=\"10\" y=\"10\" width=\"1180\" height=\"1080\" \
         \       fill=\"none\" stroke=\"red\" /> "
svgFtr = "</svg>\n"
          
ppLine :: Line -> String
ppLine (x,y,x',y') = "<path d=\"M "++ppPos x y++" L "++ppPos x' y'++"\" "++
                     "stroke=\"blue\" stroke-width=\"5\" />\n"

ppPos :: Int -> Int -> String
ppPos x y = show (50+factor*x)++" "++show (yMax-50-factor*y)

