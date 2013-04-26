package cs162_lab_6;

import java.awt.*;
import java.io.*;

public class Fibonacci {
    public Fibonacci() {}

    private int findNth(int n) {
        if (n < 1) {
            System.out.println("Need positive integral input!");
            System.exit(-1);
        }
        else if (n < 3) {
            return 1;
        }
        else {
            return findNth(n-1) + findNth(n-2);
        }
        return 0;
    }

    public static void main(String[] args) {
        Fibonacci fib = new Fibonacci();

        for (int i=1; i<20; i++) {
            System.out.println(fib.findNth(i));
        }
    }
}

