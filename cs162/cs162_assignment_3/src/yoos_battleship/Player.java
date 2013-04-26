/*! \file Player.java
 *  \author Soo-Hyun Yoo
 *  \brief Abstract class for players.
 */

package yoos_battleship;

import java.awt.*;
import java.awt.event.*;

import yoos_battleship.BattleshipBoard;

abstract class Player {
    int clearedCells[][];   // Cells that have already been shot at. Initialized to all -1.
    int nextCol, nextRow, prevCol, prevRow;

    /*! Make a shot.
     *
     *  \param board Board to shoot on.
     *  \param col Column of shot.
     *  \param row Row of shot.
     *
     *  \return Result of shot. 1 is a hit, 0 is a miss.
     */
    protected int shoot(BattleshipBoard board, int col, int row) {
        if (board.fireShot(col, row)) {
            clearedCells[col][row] = 1;
        }
        else {
            clearedCells[col][row] = 0;
        }
        return clearedCells[col][row];
    }

    // Return column of last target.
    public int getPrevTargetCol() {
        return prevCol;
    }

    // Return row of last target.
    public int getPrevTargetRow() {
        return prevRow;
    }

    public Player() {
        clearedCells = new int[10][10];
        nextCol = 0;
        nextRow = 0;

        for (int i=0; i<10; i++) {
            for (int j=0; j<10; j++) {
                clearedCells[i][j] = -1;
            }
        }
    }

    // Currently, play(BattleshipBoard, int, int) is only used by HumanPlayer.
    abstract public int play(BattleshipBoard board);
    abstract public int play(BattleshipBoard board, int col, int row);
}

