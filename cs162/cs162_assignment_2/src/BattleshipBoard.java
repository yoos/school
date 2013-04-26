/**
 * This class represents a board in the classic game of Battleship.
 */

import java.util.ArrayList;

public class BattleshipBoard {

	/** Static constant to indicate a board location is empty */
	public static final char EMPTY = '.';

	/** The 2D array representing the board */
	private char board[][];

	/** The number of battleships left */
	private int numBattleshipsLeft;

	/** The ID of the current ship being placed */
	private int currentShipID;

	/** The number of columns */
	private int numCols;

	/** The number of rows */
	private int numRows;

	/** Maintains information about the shots taken by each ship */
	private ArrayList<Double> shipInfo;

	/**
	 * Constructor for the Battleship Board. It allows for boards of arbitrary
	 * rows and columns to be created
	 * 
	 * 
	 * @param numRows
	 *            The number of rows
	 * @param numCols
	 *            The number of columns
	 * @throw BattleshipException if numRows <= 0 or numCols <= 0
	 */
	public BattleshipBoard(int numRows, int numCols) throws BattleshipException {
		if (numRows <= 0) {
			throw new BattleshipException("Number of rows must be >= 0");
		}
		if (numCols <= 0) {
			throw new BattleshipException("Number of columns must be >= 0");
		}
		this.numCols = numCols;
		this.numRows = numRows;
		this.board = new char[numCols][numRows];
		for (int col = 0; col < numCols; col++) {
			for (int row = 0; row < numRows; row++) {
				this.board[col][row] = this.EMPTY;
			}
		}
		this.numBattleshipsLeft = 0;
		this.currentShipID = 0;
		this.shipInfo = new ArrayList<Double>();
	}

	/**
	 * Getter for the number of columns in the board
	 * 
	 * @return The number of columns in the Battleship board
	 */
	public int getNumCols() {
		return this.numCols;
	}

	/**
	 * Getter for the number of rows in the board
	 * 
	 * @return The number of rows in the Battleship board
	 */
	public int getNumRows() {
		return this.numRows;
	}

	/**
	 * Checks if the start and end row positions are a vertical placement of the
	 * battleship
	 * 
	 * @param startRow
	 *            The starting row position of the battleship
	 * @param endRow
	 *            The ending row position of the battleship
	 * @return true if it is a vertical placement, and false otherwise
	 */
	private boolean isVerticalPlacement(int startRow, int endRow) {
		if (startRow == endRow) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Checks if the start and end col positions are a horizontal placement of
	 * the battleship
	 * 
	 * @param startCol
	 *            The starting col position of the battleship
	 * @param endCol
	 *            The ending col position of the battleship
	 * @return true if it is a horizontal placement, and false otherwise
	 */
	private boolean isHorizontalPlacement(int startCol, int endCol) {
		if (startCol == endCol) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Checks of the ship can be legally placed without overlapping with
	 * existing ships
	 * 
	 * @param startCol
	 *            The starting col coordinate
	 * @param startRow
	 *            The starting row coordinate
	 * @param endCol
	 *            The ending col coordinate
	 * @param endRow
	 *            The ending row coordinate
	 * @return true if the ship doesn't overlap, false otherwise
	 */
	private boolean checkShipNoOverlap(int startCol, int startRow, int endCol,
			int endRow) {
		// Do a check horizontally first
		if (startCol == endCol) {
			for (int row = startRow; row <= endRow; row++) {
				if (this.board[startCol][row] != this.EMPTY) {
					return false;
				}
			}
		} else if (startRow == endRow) {
			for (int col = startCol; col <= endCol; col++) {
				if (this.board[col][startRow] != this.EMPTY) {
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * Places the battleship at the starting coordinates (startCol,startRow) and
	 * the end coordinates (endCol,endRow) inclusive.
	 * 
	 * @param startCol
	 *            The starting Col coordinate
	 * @param startRow
	 *            The starting Row coordinate
	 * @param endCol
	 *            The ending Col coordinate
	 * @param endRow
	 *            The ending Row coordinate
	 * @throws IndexOutOfBoundsException
	 *             If the coordinates are out of bounds. A coordinate is in
	 *             bounds if 0 <= col < (number of columns - 1) and 0 <= row <
	 *             (number of rows - 1).
	 * @throws BattleshipException
	 *             If ship is placed diagonally If it overlaps with another
	 *             battleship If startCol > endCol or startRow > endRow.
	 */
	public void placeShip(int startCol, int startRow, int endCol, int endRow)
			throws BattleshipException, IndexOutOfBoundsException {
		int shipSize = 0;
		if (!isHorizontalPlacement(startCol, endCol)
				&& !isVerticalPlacement(startRow, endRow)) {
			throw new BattleshipException("Cannot place battleship diagonally");
		}

		if (!checkShipNoOverlap(startCol, startRow, endCol, endRow)) {
			throw new BattleshipException(
					"Battleship overlaps another battleship");
		}

		if ((startCol > endCol) || (startRow > endRow)) {
			throw new BattleshipException("Invalid starting coordinate");
		}

		// Then place ship
		if (startCol == endCol) {
			for (int row = startRow; row <= endRow; row++) {
				this.board[startCol][row] = Character.forDigit(currentShipID,
						10);
				shipSize++;
			}
		} else if (startRow == endRow) {
			for (int col = startCol; col <= endCol; col++) {
				this.board[col][startRow] = Character.forDigit(currentShipID,
						10);
				shipSize++;
			}
		}
		this.shipInfo.add(new Double(shipSize));
		this.currentShipID++;
		this.numBattleshipsLeft++;
	}

	/**
	 * Fires a shot at a battleship
	 * 
	 * @param col
	 *            The col coordinate of the shot
	 * @param row
	 *            The row coordinate of the shot
	 * @return true if an enemy battleship is hit, false otherwise
	 * @throws IndexOutOfBoundsException
	 *             if col or row are out of bounds
	 */
	public boolean fireShot(int col, int row) throws IndexOutOfBoundsException {
		if (this.board[col][row] != this.EMPTY) {
			int shipID = Character.digit(this.board[col][row], 10);
			Double shotsLeft = this.shipInfo.get(shipID);
			shotsLeft--;
			if (shotsLeft == 0) {
				this.numBattleshipsLeft--;
			}
			this.shipInfo.set(shipID, new Double(shotsLeft));
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Prints the board
	 */
	public void print() {
		for (int row = 0; row < this.numRows; row++) {
			StringBuffer sb = new StringBuffer(this.numCols);
			for (int col = 0; col < this.numCols; col++) {
				sb.append(this.board[col][row]);
			}
			System.out.println(sb.toString());
		}
	}

	/**
	 * Main function
	 * 
	 * @param args
	 *            Command line arguments
	 */
	public static void main(String args[]) {
		try {
			BattleshipBoard b = new BattleshipBoard(10, 10);
			b.placeShip(1, 1, 1, 3);
			b.placeShip(4, 4, 6, 4);
			b.placeShip(6, 2, 6, 3);
			b.fireShot(6, 2);
			b.fireShot(6, 3);
			b.fireShot(6, 4);
			b.fireShot(4, 4);
			b.fireShot(5, 4);
			b.fireShot(6, 4);
			b.fireShot(1, 1);
			b.fireShot(1, 2);
			b.fireShot(1, 3);
			b.print();
			System.out.println(b.isGameOver());
		} catch (Exception e) {
			System.out.println(e.getMessage());
			System.exit(-1);
		}
	}

	/*****************************************
	 * Everything below is FOR BONUS ONLY
	 *****************************************/

	/**
	 * FOR BONUS ONLY: Gets the number of Battleships left
	 * 
	 * @return The number of battleships left
	 */
	public int getNumBattleshipsLeft() {
		return this.numBattleshipsLeft;
	}

	/**
	 * FOR BONUS ONLY: Returns true if game is over, false otherwise
	 * 
	 * @return true if game is over, false otherwise
	 */
	public boolean isGameOver() {
		if (this.numBattleshipsLeft <= 0) {
			return true;
		} else {
			return false;
		}
	}

}
