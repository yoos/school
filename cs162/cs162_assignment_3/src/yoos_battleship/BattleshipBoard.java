
package yoos_battleship;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
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

	/** The number of rows */
	private int numRows;

	/** The number of columns */
	private int numCols;

	/** Maintains information about the shots taken by each ship */
	private ArrayList<Integer> shipInfo;

	/**
	 * Constructor for the Battleship Board. It allows for boards of arbitrary
	 * rows and columns to be created. The board must be empty initially (before
	 * any ships are placed).
	 * 
	 * @param numRows
	 *            The number of rows
	 * @param numCols
	 *            The number of columns
	 * @throws BattleshipException if numRows or numCols are <= 0
	 */
	public BattleshipBoard(int numRows, int numCols) throws BattleshipException {
		if( numRows <= 0 ) {
			throw new BattleshipException("Invalid number of rows");
		}
		if( numCols <= 0 ) {
			throw new BattleshipException("Invalid number of columns");
		}
		this.numRows = numRows;
		this.numCols = numCols;
		this.board = new char[numCols][numRows];
		for (int c = 0; c < numCols; c++) {
			for (int r = 0; r < numRows; r++) {
				this.board[c][r] = BattleshipBoard.EMPTY;
			}
		}
		this.numBattleshipsLeft = 0;
		this.currentShipID = 0;
		this.shipInfo = new ArrayList<Integer>();
	}

	/**
	 * Constructor for the Battleship Board.  
	 * File must have 10 lines, with exactly 10 characters per line.
	 * '.' represents EMPTY.  A unique number (0, 1, 2, 3, or 4) will represent each ship.
	 * Example File:
	 * ..........
	 * ..........
	 * ...11111..
	 * ..02......
	 * ..02......
	 * ..02......
	 * ...2......
	 * ...33344..
	 * ..........
	 * ..........
	 * 
	 * @param boardFile a text file with a 10x10 character grid, representing a battleship board
	 * @throws FileNotFoundException if the board file is not found
	 * @author richmaja
	 */
	public BattleshipBoard(File boardFile) throws FileNotFoundException{
		this.numRows = 10;
		this.numCols = 10;
		this.board = new char[numCols][numRows];
		this.shipInfo = new ArrayList<Integer>();
		this.numBattleshipsLeft = 5;
		//Add 5 entries to shipInfo
		for(int i=0; i < 5; i++){
			shipInfo.add(0);
		}
		
		try{
			if(!boardFile.exists()) throw new FileNotFoundException("Error Reading: "+boardFile.getName());

			FileReader fr = new FileReader(boardFile);
			BufferedReader br = new BufferedReader(fr);
			String line;
			int lineCounter = 0;
			while((line = br.readLine())!=null){
				for(int i = 0; i < line.length(); i++){
					char currentCharacter = line.charAt(i);
					board[i][lineCounter] = currentCharacter;
				}
				lineCounter++;
			}
			br.close();
			fr.close();

			for(int c = 0; c < this.numCols; c++){
				for(int r = 0; r < this.numRows; r++){
					try{
						int shipIdVal = Integer.parseInt(board[c][r]+"");
						shipInfo.set(shipIdVal, shipInfo.get(shipIdVal)+1);
					}catch(NumberFormatException e){}
				}
			}
		} catch(IOException e){
			System.out.println("Error: "+e.getMessage());
		}

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
	 * Getter for the number of columns in the board
	 * 
	 * @return The number of columns in the Battleship board
	 */
	public int getNumCols() {
		return this.numCols;
	}

	/**
	 * Gets the number of Battleships left
	 * 
	 * @return The number of battleships left
	 */
	public int getNumBattleshipsLeft() {
		return this.numBattleshipsLeft;
	}

	/**
	 * Checks if the cell is occupied.
	 * @param col The column
	 * @param row The row
	 * @return true if the cell is occupied
	 */
	public boolean isOccupied(int col, int row){
		if(row >= 0 && row < getNumRows() && col >= 0 && col < getNumCols()){
			return board[col][row]!=EMPTY;
		}
		return false;
	}
	
	/**
	 * Returns the char value at a certain row and column of the board
	 * @param col The column on which the cell resides
	 * @param row The row on which the cell resides
	 * @return the char value at a certain row and column of the board
	 */
	public char getCellContent(int col, int row){
		if(row >= 0 && row < getNumRows() && col >= 0 && col < getNumCols()){
			return board[col][row];
		}
		return EMPTY;
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
	 * Checks if the start and end col positions are a horizontal placement of the
	 * battleship
	 * 
	 * @param startCol
	 *            The starting Col position of the battleship
	 * @param endCol
	 *            The ending Col position of the battleship
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
	 * @param startColumn
	 *            The starting Column coordinate
	 * @param startRow
	 *            The starting Row coordinate
	 * @param endColumn
	 *            The ending Column coordinate
	 * @param endRow
	 *            The ending Row coordinate
	 * @return true if the is a legal placement, false otherwise
	 */
	private boolean checkShipPlacement(int startColumn, int startRow, int endColumn,
			int endRow) {
		// Do a check first
		if (startRow == endRow) {
			for (int c = startColumn; c <= endColumn; c++) {
				if (this.board[c][startRow] != BattleshipBoard.EMPTY) {
					return false;
				}
			}
		} else if (startColumn == endColumn) {
			for (int r = startRow; r <= endRow; r++) {
				if (this.board[startColumn][r] != BattleshipBoard.EMPTY) {
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * Places the battleship at the starting coordinates (startCol,startRow) and the
	 * end coordinates (endCol,endRow) inclusive. This means that if you place a ship
	 * at (1,3) to (1,5), it is placed on (1,3), (1,4) AND (1,5). Size 1 ships are
	 * allowed to be placed.
	 * 
	 * @param startColumn
	 *            The starting Column coordinate
	 * @param startRow
	 *            The starting Row coordinate
	 * @param endColumn
	 *            The ending Column coordinate
	 * @param endRow
	 *            The ending Row coordinate
	 * @throws IndexOutOfBoundsException 	 Thrown if the coordinates are out of bounds. 
	 *            The row coordinate is in bounds if 0 <= row < (number of rows - 1). 
	 *            The col coordinate is in bounds if 0 <= col < (number of columns - 1).  
	 * 		   BattleshipException
	 *             If ship is placed diagonally
	 *             If it overlaps with another battleship  
	 *             If startRow > endRow or startColumn > endColumn. 
	 */
	public void placeShip(int startColumn, int startRow, int endColumn, int endRow)
	throws BattleshipException, IndexOutOfBoundsException {
		int shipSize = 0;
		if (!isHorizontalPlacement(startColumn, endColumn)
				&& !isVerticalPlacement(startRow, endRow)) {
			throw new BattleshipException("Cannot place battleship diagonally");
		}

		if (!checkShipPlacement(startColumn, startRow, endColumn, endRow)) {
			throw new BattleshipException("Battleship overlaps another battleship");
		}

		if( startRow > endRow ) {
			throw new BattleshipException("Invalid startRow coordinates");
		}

		if( startColumn > endColumn ) {
			throw new BattleshipException("Invalid startColumn coordinates");
		}

		// Then place ship
		if (startRow == endRow) {
			for (int c = startColumn; c <= endColumn; c++) {
				this.board[c][startRow] = Character.forDigit(currentShipID,10);
				shipSize++;
			}
		} else if (startColumn == endColumn) {
			for (int r = startRow; r <= endRow; r++) {
				this.board[startColumn][r] = Character.forDigit(currentShipID,10);
				shipSize++;
			}
		}
		this.shipInfo.add(new Integer(shipSize));
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
	 * @return true if an enemy battleship is hit, false otherwise. If you fire twice at the same
	 * 	       spot and a battleship was at that spot, this method returns true both times.
	 * @throws IndexOutOfBoundsException
	 *             if row or col are out of bounds
	 */
	public boolean fireShot(int col, int row) throws IndexOutOfBoundsException {
		if (this.board[col][row] != BattleshipBoard.EMPTY) {
			int shipID = Character.digit(this.board[col][row],10);
			Integer shotsLeft = this.shipInfo.get(shipID);
			shotsLeft--;
			if( shotsLeft == 0 ) {
				this.numBattleshipsLeft--;
			}
			this.shipInfo.set(shipID,new Integer(shotsLeft));				
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Prints the board
	 */
	public void print() {
		for (int r = 0; r < this.numRows; r++) {
			StringBuffer sb = new StringBuffer(this.numCols);
			for (int c = 0; c < this.numCols; c++) {
				sb.append(this.board[c][r]);
			}
			System.out.println(sb.toString());
		}
	}

	/**
	 * Returns true if game is over, false otherwise
	 * @return true if game is over, false otherwise
	 */
	public boolean isGameOver() {
		if( this.numBattleshipsLeft <= 0 ) {
			return true;
		} else {
			return false;
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
			System.out.println("My Main Method Is Stupid.  It Does NOTHING");
		} catch (Exception e) {
			e.printStackTrace();
			//System.out.println(e.getMessage());
			System.exit(-1);
		}
	}
}
