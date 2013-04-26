

public class CommandLinePrinter {
    public static void main(String[] args) {
        System.out.println("Command line arguments:");
        for (int i=0; i<args.length; i++) {
            System.out.println("Arg " + i + ": " + args[i]);
        }
    }
}

