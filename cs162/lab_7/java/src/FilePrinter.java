
import java.io.*;

public class FilePrinter {
    public static void main(String[] args) {
        File[] list = (new File(args[0])).listFiles();
        for (int i=0; i<list.length; i++) {
            if (list[i].isDirectory()) {
                System.out.println(list[i].toString());
            }
        }
        for (int i=0; i<list.length; i++) {
            if (list[i].isFile()) {
                System.out.println(list[i].toString());
            }
        }
    }
}

