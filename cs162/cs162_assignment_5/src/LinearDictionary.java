
import java.awt.*;
import java.io.*;
import java.util.*;

public class LinearDictionary extends Dictionary {
    ArrayList<String> words = new ArrayList<String>();   // List of words

    public LinearDictionary(File wordList) throws FileNotFoundException {
        try {
            if (!wordList.exists()) throw new FileNotFoundException("Error reading: " + wordList.getName());

            FileReader fr = new FileReader(wordList);
            BufferedReader br = new BufferedReader(fr);
            String word;

            // Populate list of words.
            while ((word = br.readLine()) != null) {
                words.add(word);
            }

            br.close();
            fr.close();
        }
        catch (IOException e) {
            System.out.println("Error: " + e.getMessage());
        }
    }

    // Linear search.
    public boolean contains(String word) {
        long time = System.nanoTime();
        for (String each : words) {
            if (each.equals(word)) {
                avgLookupTime = avgLookupTime * (avgIndex-1)/avgIndex + (System.nanoTime() - time)/avgIndex;
                avgIndex++;
                return true;
            }
        }
        return false;
    }
}

