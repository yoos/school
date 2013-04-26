/*! \file Battleship.java
 *  \author Soo-Hyun Yoo
 *  \brief Battleship main executable.
 */

package yoos_battleship;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;

import yoos_battleship.BattleshipGUI;
import yoos_battleship.BattleshipException;

public class Battleship {
    public Battleship(String args[]) {
    }

    public static void main(String args[]) {
        System.out.println("Starting Battleship!");
        try {
            // Check that three arguments were provided.
            if (args.length != 3) {
                throw new BattleshipException("You must provide three arguments!");
            }

            // Load GUI.
            try {
                JFrame gui = new BattleshipGUI(new BattleshipBoard(new File(args[0])),
                                               new BattleshipBoard(new File(args[1])),
                                               args[2].charAt(0));

            }
            catch (FileNotFoundException e) {
                System.out.println(e.getMessage());
                System.exit(-1);
            }
        }
        catch (BattleshipException e) {
            System.out.println(e.getMessage());
            System.exit(-1);
        }
        catch (Exception e) {
            e.printStackTrace();
            //System.out.println(e.getMessage());
            System.exit(-1);
        }
    }
}
