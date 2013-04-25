/**
 * 
 */
package edu.oregonstate.eecs.cs331.assn2;

/**
 * This is the interface for the game player.
 * @author Chris Ventura
 *
 */
public interface Player {

	/**
	 * These are constants to define the type of the player.
	 */
	public static final int HUMAN_PLAYER = 0;
	public static final int RANDOM_PLAYER = 1;
	public static final int MINIMAX_PLAYER = 2;
	public static final int ALPHABETA_PLAYER = 3;
	
	/**
	 * Does the next action given the current board state.
	 * @param state The current board state
	 * @return The next action to do.
	 * @throws Exception
	 */
	public Position getNextMove(TicTacToeBoard state) throws Exception;
	
	/**
	 * Gets the player type, where the player type can take on the values in the constants of the player
	 * interface.
	 * @return The player type
	 */
	public int getPlayerType();
}
