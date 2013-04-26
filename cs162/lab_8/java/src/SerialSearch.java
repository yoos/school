
import java.awt.*;
import java.util.Random;

public class SerialSearch {
    Random randgen = new Random();

    static int size = 10000000;
    static int range = 1000000000;

    int array[] = new int[size];

    private int search(int value) {
        int i=0;

        while (array[i] != value && i<size) {
            i++;
        }

        if (i >= size) {
            return -1;
        }
        else {
            return i;
        }
    }

    public int searchRandom() {
        int index = randgen.nextInt(size);

        return search(array[index]);
    }

    public SerialSearch() {
        for (int i=0; i<size; i++) {
            array[i] = randgen.nextInt(range);
        }
    }

    public static void main(String[] args) {
        SerialSearch ss = new SerialSearch();

        for (int i=0; i<500; i++) {
            int numSearched = ss.searchRandom();
            if (numSearched == -1) {
                System.exit(-1);
            }
            else {
                System.out.println(numSearched);
            }
        }
    }
}

