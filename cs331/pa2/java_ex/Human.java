package edu.oregonstate.eecs.cs331.assn2;

/**
 * This class defines the Player Interface implemenation for human players. 
 * @author Chris Ventura
 *
 */
public class Human implements Player {

	/**
	 * This constant is used to specify that a square is invalid.
	 */
	private static final int INVALID = -1;
	
	/**
	 * The square that the player has chosen.
	 */
	private Position chosenSquare;
	
	/**
	 * Flag specifying whether the human player has taken his/her action
	 * in the game.
	 */
	private boolean actionTaken;

	/**
	 * The constructor for the Human class. It sets the default settings.
	 *
	 */
	public Human() {
		chosenSquare = new Position();
		chosenSquare.row = INVALID;
		chosenSquare.col = INVALID;
		actionTaken = true;
	}
	
	/**
	 * Returns the next move to do.
	 * @param state The current board state
	 * @return The next move to do.
	 */
	public Position getNextMove(TicTacToeBoard state) throws Exception {
		/* Keep waiting until the user has actually taken action */
		while(getActionTaken()) {
			
		}
		setActionTaken(true);
		
		return chosenSquare;
	}

	/**
	 * Returns the player type.
	 * @return The player type.
	 */
	public int getPlayerType() {
		return HUMAN_PLAYER;
	}
	
	/**
	 * Sets which square the user has actually chosen.
	 * @param row The row of the chosen square.
	 * @param col The column of the chosen square.
	 */
	public synchronized void setChosenSquare(int row, int col){
		if(actionTaken) {
			chosenSquare.row = row;
			chosenSquare.col = col;
			actionTaken = false;
		}
	}

	/**
	 * Retrieves the actionTaken flag.
	 * @return Whether the user has acted or not.
	 */
	private synchronized boolean getActionTaken() {
		return actionTaken;
	}
	
	/**
	 * Sets the actionTaken flag.
	 * @param value
	 */
	private synchronized void setActionTaken(boolean value) {
		actionTaken = value;
	}
}
