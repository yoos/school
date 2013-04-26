
import java.awt.*;
import java.util.Random;

public class CompareSorts {
    Random randgen = new Random();
    Sorting sort = new Sorting();

    int n10[] = new int[10];
    int n100[] = new int[100];
    int n1000[] = new int[1000];
    int n10000[] = new int[10000];
    int n100000[] = new int[100000];

    int[][] lists = {n10, n100, n1000, n10000, n100000};

    public CompareSorts() {
        for (int i=0; i<5; i++) {
            double range = 10*Math.pow(10, i);
            for (int j=0; j<range; j++) {
                lists[i][j] = randgen.nextInt((int) range);
            }
        }
    }

    public void testSelectionSort() {
        long before, after;
        System.out.println("Testing selection sort.");
        for (int i=0; i<5; i++) {
            before = System.currentTimeMillis();
            sort.selectionSort(lists[i]);
            after = System.currentTimeMillis();

            System.out.println(after-before);
        }
    }

    public void testBubbleSort() {
        long before, after;
        System.out.println("Testing bubble sort.");
        for (int i=0; i<5; i++) {
            before = System.currentTimeMillis();
            sort.bubbleSort(lists[i]);
            after = System.currentTimeMillis();

            System.out.println(after-before);
        }
    }

    public void testInsertionSort() {
        long before, after;
        System.out.println("Testing insertion sort.");
        for (int i=0; i<5; i++) {
            before = System.currentTimeMillis();
            sort.insertionSort(lists[i]);
            after = System.currentTimeMillis();

            System.out.println(after-before);
        }
    }

    public void testMergeSort() {
        long before, after;
        System.out.println("Testing merge sort.");
        for (int i=0; i<5; i++) {
            before = System.currentTimeMillis();
            sort.mergeSort(lists[i]);
            after = System.currentTimeMillis();

            System.out.println(after-before);
        }
    }

    public void testQuickSort() {
        long before, after;
        System.out.println("Testing quick sort.");
        for (int i=0; i<5; i++) {
            before = System.currentTimeMillis();
            sort.quicksort(lists[i]);
            after = System.currentTimeMillis();

            System.out.println(after-before);
        }
    }


    public static void main(String[] args) {
        CompareSorts compare = new CompareSorts();

        compare.testSelectionSort();
        compare.testBubbleSort();
        compare.testInsertionSort();
        compare.testMergeSort();
        compare.testQuickSort();
    }
}

