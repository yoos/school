/** \file BattleshipBoardTest.java
 *  \author Soo-Hyun Yoo
 *  \brief CS 162 Assignment 2
 */

import static org.junit.Assert.*;
import org.junit.Test;

public class BattleshipBoardTest {
    /** \brief Check that the constructed board is initially empty.
     */
    @Test
    public void testEmptyBoardInit() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Run fireShot() for each cell on board. They should all return false.
        for (int i=0; i<10; i++) {
            for (int j=0; j<10; j++) {
                assertEquals(false, board.fireShot(i, j));
            }
        }
    }

    /** \brief Check that the board is of the right size if a positive board is
     *  requested.
     */
    @Test
    public void testPositiveBoardSize() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Check number of columns and rows.
        assertEquals(10, board.getNumCols());
        assertEquals(10, board.getNumRows());
    }

    /** \brief Check that a BattleshipException is thrown if the number of
     *  columns is < 0.
     */
    @Test (expected = BattleshipException.class)
    public void testNegColNum() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(-2, 10);
    }

    /** \brief Check that a BattleshipException is thrown if the number of rows
     *  is < 0
     */
    @Test (expected = BattleshipException.class)
    public void testNegRowNum() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, -2);
    }

    /** \brief Check that a BattleshipException is thrown if a 0x0 board is
     *  requested.
     */
    @Test (expected = BattleshipException.class)
    public void testZeroSizeBoard() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(0, 0);
    }




    /** \brief Check that ship placement works, including 0.
     */
    @Test
    public void testShipPlacement() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 0, 1, 5);

        // Check that the appropriate cells are occupied.
        for (int i=2; i<6; i++) {
            assertEquals(true, board.fireShot(1, i));
        }
    }

    /** \brief Check that placeShip throws IndexOutOfBoundsException when
     *  column or row < 0.
     */
    @Test (expected = IndexOutOfBoundsException.class)
    public void testShipNegBoundsPlacement() throws BattleshipException, IndexOutOfBoundsException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(-1, 2, 3, 2);
    }

    /** \brief Check that placeShip throws IndexOutOfBoundsException when
     *  column > numCols-1 or row > numRows-1.
     */
    @Test (expected = IndexOutOfBoundsException.class)
    public void testShipPosBoundsPlacement() throws BattleshipException, IndexOutOfBoundsException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(8, 2, 10, 2);
    }

    /** \brief Check that placeShip throws BattleshipException if ship is
     *  placed diagonally.
     */
    @Test (expected = BattleshipException.class)
    public void testShipDiagonalPlacement() throws BattleshipException, IndexOutOfBoundsException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 1, 4, 3);
    }

    /** \brief Check that placeShip throws BattleshipException if ships
     *  overlap.
     */
    @Test (expected = BattleshipException.class)
    public void testShipOverlappingPlacement() throws BattleshipException, IndexOutOfBoundsException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 3, 5, 3);

        // Place another ship so it overlaps the first ship.
        board.placeShip(2, 2, 2, 5);
    }

    /** \brief Check that a BattleshipException is thrown if the coordinate
     *  pairs are specified in the wrong order (i.e., startCol > endCol or
     *  startRow > endRow).
     */
    @Test (expected = BattleshipException.class)
    public void testStartEndPlacementOrder() throws BattleshipException, IndexOutOfBoundsException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 5, 1, 2);
    }

    /** \brief Check that a BattleshipException is thrown (not an
     *  IndexOutOfBoundsException) if coordinate pairs are specified in the
     *  wrong order and one of the pairs are out of bounds.
     */
    @Test (expected = BattleshipException.class)
    public void testStartEndBadPlacementOrder() throws BattleshipException, IndexOutOfBoundsException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 5, 1, -2);
    }




    /** \brief fireShot() should return true if a battleship is hit.
     */
    @Test
    public void testCellHit() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 1, 1, 6);

        // Fire a shot at the ship.
        assertEquals(true, board.fireShot(1,2));
    }

    /** \brief fireShot() should return false if a battleship is not hit.
     */
    @Test
    public void testCellNotHit() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 1, 1, 6);

        // Fire a bad shot at the ship.
        assertEquals(false, board.fireShot(2,2));
    }

    /** \brief fireShot() should return true both times if a battleship is hit
     *  twice in the same spot.
     */
    @Test
    public void testCellHitTwice() throws BattleshipException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 1, 1, 6);

        // Fire a shot at the ship.
        assertEquals(true, board.fireShot(1,2));

        // Fire a second shot at the same spot.
        assertEquals(true, board.fireShot(1,2));
    }

    /** \brief fireShot() should return false if coordinates are negative.
     */
    @Test (expected = IndexOutOfBoundsException.class)
    public void testCellNeg() throws BattleshipException, IndexOutOfBoundsException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 1, 1, 6);

        // Fire a bad shot at the ship.
        assertEquals(false, board.fireShot(1, -1));
    }

    /** \brief fireShot() should return false if coordinates are generally out
     *  of bounds.
     */
    @Test (expected = IndexOutOfBoundsException.class)
    public void testCellOutOfBounds() throws BattleshipException, IndexOutOfBoundsException {
        // Initialize board.
        BattleshipBoard board = new BattleshipBoard(10, 10);

        // Place a ship.
        board.placeShip(1, 1, 1, 6);

        // Fire a bad shot at the ship.
        assertEquals(false, board.fireShot(1, 10));
    }
}
