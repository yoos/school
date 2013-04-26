import java.io.PrintStream;

/**
 * This class encapsulates all of the state information of the TicTacToe board.
 * 
 * @author Chris Ventura
 * 
 */
public class TicTacToeBoard {

	/**
	 * A symbol for the contents of a blank square.
	 */
	public static final String BLANK = "_";
	
	/**
	 * The symbol for a square occupied by player X
	 */
	public static final String X = "X";
	
	/**
	 * The symbol for a square occupied by player O
	 */
	public static final String O = "O";
	
	/**
	 * The player index for an empty square
	 */
	public static final int PLAYER_NONE = -1;
	
	/**
	 * The player index for a square occupied by X
	 */
	public static final int PLAYER_X = 0;
	
	/**
	 * The player index for a square with an O in it.
	 */
	public static final int PLAYER_O = 1;

	/**
	 * This constant specifies the number of squares in a row and the number of
	 * squares in a column.
	 */
	public static final int SIZE = 3;

	/**
	 * The state of each square.
	 */
	private String square[][];

	/**
	 * Specifies whose turn it is.
	 */
	private int turn;

	/**
	 * The number of empty squares
	 */
	private int numEmptySquares;

	/**
	 * The contructor for the class. It sets the current state of the game to
	 * the initial state of the game.
	 * 
	 */
	public TicTacToeBoard() {
		this.turn = PLAYER_X;
		this.numEmptySquares = 9;
		this.square = new String[SIZE][SIZE];
		for (int row = 0; row < SIZE; row++) {
			for (int col = 0; col < SIZE; col++) {
				this.square[row][col] = BLANK;
			}
		}
	}

	/**
	 * The method to use when needing to update the game board's state.
	 * 
	 * @param row
	 *            The row of the square to update.
	 * @param col
	 *            The column of the square to update.
	 * @param playerSymbol
	 *            The String symbol of the player who now has the square.
	 * @throws Exception If the playerSymbol is not "X" or "O"
	 */
	public void setState(int row, int col, String playerSymbol) throws Exception {
		if ((square[row][col] == BLANK) && (playerSymbol != BLANK)) {
			this.numEmptySquares--;
		}
		if( !playerSymbol.equals(TicTacToeBoard.X) && !playerSymbol.equals(TicTacToeBoard.O)) {
			throw new Exception("Invalid player symbol");
		}
		square[row][col] = playerSymbol;
	}

	/**
	 * The method to use when needing to update the game board's state.
	 * 
	 * @param row
	 *            The row of the square to update.
	 * @param col
	 *            The column of the square to update.
	 * @param playerIndex
	 *            The integer index of the player who now has the square.
	 * @throws Exception If the player index is invalid
	 */
	public void setState(int row, int col, int playerIndex) throws Exception {
		if ((square[row][col] == BLANK) && (playerIndex != PLAYER_NONE)) {
			this.numEmptySquares--;
		}
		String playerSymbol;
		if( playerIndex == PLAYER_X ) {
			playerSymbol = X;
		} else if ( playerIndex == PLAYER_O ) {
			playerSymbol = O;
		} else {
			throw new Exception("Invalid player symbol");
		}
		square[row][col] = playerSymbol;
	}

	/**
	 * Returns the contents of a square where the contents are the constants X, O, or BLANK
	 * 
	 * @param row
	 *            The row of the square
	 * @param col
	 *            The col of the square
	 * @return The contents of the square where the contents are the constants X, O, or BLANK
	 */
	public String getState(int row, int col) {
		return this.square[row][col];
	}

	/**
	 * Returns the index of the player who owns the square.  The player index can be PLAYER_X, PLAYER_O, or PLAYER_NONE.
	 * 
	 * @param row
	 *            The row of the square
	 * @param col
	 *            The col of the square
	 * @throws Exception if player symbol is invalid
	 * @return The index of the player who owns the square.
	 */
	public int getPlayerIndexOfSquare(int row, int col) throws Exception {
		String playerSymbol = this.square[row][col];
		if( playerSymbol.equals(X) ) {
			return PLAYER_X;
		} else if ( playerSymbol.equals(O) ) {
			return PLAYER_O;
		} else if ( playerSymbol.equals(BLANK) ) {
			return PLAYER_NONE;
		} else {
			throw new Exception("Invalid Player Symbol in square");
		}
	}

	/**
	 * Retrieves the player index who has the current turn.
	 * 
	 * @return The player index who has the current turn.
	 */
	public synchronized int getTurn() {
		return this.turn;
	}

	/**
	 * Sets which player has the next turn.
	 * 
	 * @param player
	 *            The player who will have the next turn.
	 */
	public synchronized void setTurn(int player) {
		this.turn = player;
	}

	/**
	 * Returns if the game is over or not
	 * @return True if the game is over and false otherwise.
	 * @throws Exception
	 */
	public boolean isGameOver() throws Exception {
		if ((this.numEmptySquares == 0) || isWin()) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Returns true if a player wins the game.
	 * @return True if a player wins the game and false if the game is a draw or it is still going on.
	 * @throws Exception
	 */
	public boolean isWin() throws Exception {
		return (isWin(PLAYER_X) || isWin(PLAYER_O));
	}

	/**
	 * Returns true if the specified player wins the game
	 * @param player The index of the player
	 * @return True if the specified player wins the game and false otherwise.
	 * @throws Exception If player is not a legal player index.
	 */
	public boolean isWin(int player) throws Exception {
		if (isVerticalWin(player) || isHorizontalWin(player)
				|| isDiagonalWin(player)) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * Checks if the specified player has three Xs or Os in a row.
	 * @param player The player index
	 * @return True if there is a vertical 3-in-a-row for the specified player and false otherwise.
	 * @throws Exception If the player index is invalid.
	 */
	private boolean isVerticalWin(int player) throws Exception {
		boolean result = false;
		String playerSymbol;
		if ((player < 0) || (player > TicTacToeBoard.PLAYER_O)) {
			throw new Exception("Invalid player index -- must be 0 or 1");
		}
		if( player == PLAYER_X) {
			playerSymbol = X;
		} else {
			playerSymbol = O;
		}
		for (int col = 0; col < SIZE; col++) {
			if ((square[0][col] == square[1][col])
					&& (square[1][col] == square[2][col])
					&& (square[2][col] == playerSymbol)) {
				result = true;
				break;
			}
		}
		return result;
	}

	/**
	 * Checks if the specified player has three Xs or Os in a row.
	 * 
	 * @param player The player index
	 * @return True if there is a horizontal 3-in-a-row for the specified player and false otherwise.
	 * @throws Exception If the player index is invalid.
	 */
	private boolean isHorizontalWin(int player) throws Exception {
		boolean result = false;
		if ((player < 0) || (player > TicTacToeBoard.PLAYER_O)) {
			throw new Exception("Invalid player index -- must be 0 or 1");
		}
		String playerSymbol;
		if( player == PLAYER_X) {
			playerSymbol = X;
		} else {
			playerSymbol = O;
		}
		for (int row = 0; row < SIZE; row++) {
			if ((square[row][0] == square[row][1])
					&& (square[row][1] == square[row][2])
					&& (square[row][0] == playerSymbol)) {
				result = true;
				break;
			}
		}
		return result;
	}

	/**
	 * Checks the specified player has three X's or O's in-a-row diagonally.
	 * @param player The player index
	 * @return True if there is a diagonal 3-in-a-row pattern for the specified player and false otherwise
	 * @throws Exception If the player index is invalid.
	 */
	private boolean isDiagonalWin(int player) throws Exception {
		boolean win = false;
		if ((player < 0) || (player > TicTacToeBoard.PLAYER_O)) {
			throw new Exception("Invalid player index -- must be 0 or 1");
		}
		String playerSymbol;
		if( player == PLAYER_X) {
			playerSymbol = X;
		} else {
			playerSymbol = O;
		}
		if ((square[0][0] == square[1][1]) && (square[1][1] == square[2][2])
				&& (square[2][2] == playerSymbol)) {
			win = true;
		}

		if ((square[0][2] == square[1][1]) && (square[1][1] == square[2][0])
				&& (square[2][0] == playerSymbol)) {
			win = true;
		}

		return win;
	}

	/**
	 * Does a deep clone of the TicTacToe board object.
	 * @return A deep clone of the TicTacToe board object.
	 */
	public Object clone() {
		TicTacToeBoard deepClone = new TicTacToeBoard();
		for (int row = 0; row < TicTacToeBoard.SIZE; row++) {
			for (int col = 0; col < TicTacToeBoard.SIZE; col++) {
				deepClone.square[row][col] = this.square[row][col];
			}
		}
		deepClone.turn = this.turn;
		deepClone.numEmptySquares = this.numEmptySquares;
		return deepClone;
	}

	/**
	 * Prints the board to a PrintStream.
	 * @param out The PrintStream you want to print the board to.
	 */
	public void dump(PrintStream out) {
		for (int row = 0; row < SIZE; row++) {
			for (int col = 0; col < SIZE; col++) {
				out.print(square[row][col]);
				if( col == (SIZE-1)) {
					out.print("\n");
				} else {
					out.print(",");
				}
			}
			out.flush();
		}
		out.print("\n\n");
		out.flush();
	}
}
