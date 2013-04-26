
import java.awt.*;
import java.io.*;
import java.util.*;

public class HashDictionary extends Dictionary {
    Hashtable words = new Hashtable();   // List of words

    public HashDictionary(File wordList) throws FileNotFoundException {
        try {
            if (!wordList.exists()) throw new FileNotFoundException("Error reading: " + wordList.getName());

            FileReader fr = new FileReader(wordList);
            BufferedReader br = new BufferedReader(fr);
            String word;

            // Populate list of words.
            while ((word = br.readLine()) != null) {
                words.put(word, word);
            }

            br.close();
            fr.close();
        }
        catch (IOException e) {
            System.out.println("Error: " + e.getMessage());
        }
    }

    // Hash search.
    public boolean contains(String word) {
        Enumeration words = this.words.keys();

        long time = System.nanoTime();

        while (words.hasMoreElements()) {
            if (word.equals(words.nextElement())) {
                avgLookupTime = avgLookupTime * (avgIndex-1)/avgIndex + (System.nanoTime() - time)/avgIndex;
                avgIndex++;
                return true;
            }
        }
        return false;
    }
}

