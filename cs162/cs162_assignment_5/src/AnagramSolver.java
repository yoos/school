
import java.awt.*;
import java.io.*;
import java.util.*;

public class AnagramSolver {
    ArrayList<String> possibleWords = new ArrayList<String>();
    Dictionary dict;

    // Recursively find permutations of given string of characters.
    private ArrayList<String> permutations(String word) {
        ArrayList<String> list = new ArrayList<String>();

        if (word.length() == 1) {
            list.add(word);
            return list;
        }

        for (int i=0; i<word.length(); i++) {
            String shorter = word.substring(0, i) + word.substring(i+1);

            ArrayList<String> sublist = permutations(shorter);

            for (String s : sublist) {
                list.add(word.charAt(i) + s);
            }
        }

        return list;
    }

    public AnagramSolver(String anagram, String pathToFile, String type) throws FileNotFoundException {
        if (type.equals("l")) {
            System.out.println("Using linear dictionary.");
            dict = new LinearDictionary(new File(pathToFile));
        }
        else if (type.equals("b")) {
            System.out.println("Using binary dictionary.");
            dict = new BinaryDictionary(new File(pathToFile));
        }
        else if (type.equals("h")) {
            System.out.println("Using hash dictionary.");
            dict = new HashDictionary(new File(pathToFile));
        }
        else {
            System.out.println("Specify dictionary type: l, b, or h.");
            System.exit(-1);
        }

        for (String each : permutations(anagram)) {
            possibleWords.add(each);
        }
    }

    public String solve() {
        for (String word : possibleWords) {
            System.out.println("Searching for: " + word);
            if (dict.contains(word)) {
                System.out.println(dict.getAverageLookupTime());
                return word;
            }
        }
        return null;
    }

    public static void main(String[] args) throws FileNotFoundException {
        AnagramSolver as = new AnagramSolver(args[0], args[1], args[2]);
        for (int i=0; i<10; i++) {
            System.out.println("Anagram found: " + as.solve());
        }
    }
}

