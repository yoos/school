/*! \file HumanPlayer.java
 *  \author Soo-Hyun Yoo
 *  \brief Human player.
 */

package yoos_battleship;

public class HumanPlayer extends Player {
    // No difference between Player and HumanPlayer for now.
    public HumanPlayer() {
    }

    // Needs col/row input.
    public int play(BattleshipBoard board) {
        System.out.println("HumanPlayer needs an input!");
        return -1;
    }

    public int play(BattleshipBoard board, int col, int row) {
        prevCol = col;
        prevRow = row;
        return shoot(board, col, row);
    }
}

