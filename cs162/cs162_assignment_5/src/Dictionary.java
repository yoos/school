
import java.awt.*;
import java.io.*;
import java.util.*;

abstract class Dictionary {
    Object words;
    double avgLookupTime = 0;
    int avgIndex = 1;

    abstract public boolean contains(String word);

    public double getAverageLookupTime() {
        return avgLookupTime;
    }
}

