/*! \file RandomPlayer.java
 *  \author Soo-Hyun Yoo
 *  \brief Fire shots randomly.
 */

package yoos_battleship;

import java.util.Random;

public class RandomPlayer extends Player {
    Random randgen;

    public RandomPlayer() {
        randgen = new Random();
    }

    // Recursively cycle through cell indices (defined as colNum*10 + rowNum =
    // some integer in [0, 99]) and return the index of the cell as soon as it
    // finds the cell is -1. During normal gameplay, StackOverflowError will
    // never be thrown because the game will be terminated after at most all
    // cells have been toggled.
    private int returnNextCellIndex(int cellIndex) throws StackOverflowError {
        if (clearedCells[cellIndex / 10][cellIndex % 10] < 0) {
            return cellIndex;
        }
        else {
            return returnNextCellIndex((cellIndex+1) % 100);
        }
    }

    public int play(BattleshipBoard board) {
        try {
            int index = returnNextCellIndex(randgen.nextInt(100));
            nextCol = index / 10;
            nextRow = index % 10;
        }
        catch (StackOverflowError e) {
            System.out.println(e.getMessage());
            System.out.println("All cells have been processed. The game should be over by now!");
        }

        System.out.println("Random shot: " + nextCol + " " + nextRow);

        // In case nextCol and nextRow are modified after the call to shoot(),
        // set prevCol and prevRow.
        prevCol = nextCol;
        prevRow = nextRow;

        // Return result of shot (0 or 1).
        return shoot(board, nextCol, nextRow);
    }

    // Nothing to do with specific col/row input.
    public int play(BattleshipBoard board, int col, int row) {
        return -1;
    }
}

