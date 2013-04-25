package edu.oregonstate.eecs.cs331.assn2;

/**
 * A simple class to store the position of a move.
 * @author wong
 *
 */
public class Position {
	/**
	 * The row index of the move
	 */
	public int row;
	
	/**
	 * The column index of the move.
	 */
	public int col;
	
	/** 
	 * Default constructor
	 * 
	 */
	public Position() {
		this.row = -1;
		this.col = -1;
	}
	
	/**
	 * A constructor in which you can specify the row and column index.
	 * @param row The row index
	 * @param col The column index
	 */
	public Position(int row, int col) {
		this.row = row;
		this.col = col;
	}
}
