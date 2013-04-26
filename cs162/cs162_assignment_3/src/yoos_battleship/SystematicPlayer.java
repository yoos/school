/*! \file SystematicPlayer.java
 *  \author Soo-Hyun Yoo
 *  \brief Start at (0, 0) and fire one cell over, left to right then top to bottom.
 */

package yoos_battleship;

public class SystematicPlayer extends Player {
    public SystematicPlayer() {
        nextCol = -1;
    }

    public int play(BattleshipBoard board) {
        try {
            if (nextCol < 9) {
                nextCol++;
            }
            else {
                nextCol = 0;
                nextRow++;
            }
        }
        catch (ArrayIndexOutOfBoundsException e) {
            System.out.println(e.getMessage());
            System.out.println("All cells have been processed. The game should be over by now!");
        }

        System.out.println("Systematic shot: " + nextCol + " " + nextRow);

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

