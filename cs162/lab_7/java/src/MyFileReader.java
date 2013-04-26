
import java.io.*;
import java.util.*;

public class MyFileReader {
    public static void main(String[] args) throws FileNotFoundException {
        FileReader reader = new FileReader(args[0]);
        Scanner in = new Scanner(reader);
        for (int i=0; i<4; i++) {
            for (int j=0; j<4; j++) {
                System.out.print(in.nextLine() + " ");
            }
            System.out.print("\n");
        }
    }
}

