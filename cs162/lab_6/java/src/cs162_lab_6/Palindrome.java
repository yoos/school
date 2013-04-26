package cs162_lab_6;

import java.awt.*;
import java.io.*;

public class Palindrome {
    public Palindrome() {}

    private boolean isPalindrome(String str) {
        int len = str.length();

        if (len < 2) {
            return true;
        }
        else {
            if (str.charAt(0) == str.charAt(len-1)) {
                return isPalindrome(str.substring(1,len-1));
            }
            else {
                return false;
            }
        }
    }

    public static void main(String[] args) {
        Palindrome pal = new Palindrome();

        System.out.println(pal.isPalindrome("civic"));
        System.out.println(pal.isPalindrome("deed"));
        System.out.println(pal.isPalindrome("kayak"));
        System.out.println(pal.isPalindrome("deleveled"));
        System.out.println(pal.isPalindrome("aibohphobia"));

        System.out.println(pal.isPalindrome(""));

        System.out.println(pal.isPalindrome("not a palindrome"));
        System.out.println(pal.isPalindrome("eclipse"));
    }
}

