
import java.awt.*;
import java.io.*;
import java.util.*;

public class BinaryDictionary extends Dictionary {
    ArrayList<String> words = new ArrayList<String>();   // List of words

    public BinaryDictionary(File wordList) throws FileNotFoundException {
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

    // Binary search.
    public boolean contains(String word) {
        int low = 0;
        int high = words.size() - 1;

        long time = System.nanoTime();

        while (low <= high) {
            int mid = (low+high) / 2;
            int diff = words.get(mid).compareTo(word);
            if (diff == 0) {
                avgLookupTime = avgLookupTime * (avgIndex-1)/avgIndex + (System.nanoTime() - time)/avgIndex;
                avgIndex++;
                return true;
            }
            else if (diff < 0) {
                low = mid+1;
            }
            else {
                high = mid-1;
            }
        }
        return false;
    }
}

