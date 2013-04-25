package edu.oregonstate.eecs.cs331.assn2;

import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import javax.swing.*;

/**
 * This class is used to manage the JPanel that will contain the
 * X's and O's. It is also used to trap the mouse event and determine
 * which cell/grid to update.
 * @author Chris Ventura
 *
 */
public class CustomPanel extends JPanel {
	
	/**
	 * This is to keep the compiler happy.
	 */
	private static final long serialVersionUID = 1L;
	
	/**
	 * This is the logical position of the JPanel
	 */
	private Position location;
	
	/**
	 * This is a flag denoting if a player has already chosen
	 * this cell/grid.
	 */
	private boolean chosen; 
	
	/**
	 * Reference to the player objects to signal the human classes
	 * (if there are human players) that a cell/grid was chosen.
	 */
	private Player player1, player2;
	
	/**
	 * Reference to the state class so the JPanel is aware of whose
	 * turn it is.
	 */
	private TicTacToeBoard turnState;
	
	/**
	 * Constructor used to initialize all of the JPanels state information,
	 * and add the mouseClicked event handler.
	 * @param row The logical row of the JPanel
	 * @param col The logical column of the JPanel
	 * @param p1 The reference to player1
	 * @param p2 The reference to player2
	 * @param state The reference to the GameState class
	 */
	public CustomPanel(int row, int col, Player p1, Player p2, TicTacToeBoard state) {
		location = new Position();
		
		location.row = row;
		location.col = col;
		chosen = false;
		player1 = p1;
		player2 = p2;
		turnState = state;
		
		addMouseListener(
				new MouseAdapter() {
					/**
					 * This is the mouseClicked event handler. It passes information
					 * about which JPanel was chosen to the appropriate human player.
					 * @param event The reference to the MouseEvent information 
					 */
					public void mouseClicked (MouseEvent event) {
						if(!CustomPanel.this.chosen) {
							if((turnState.getTurn() == TicTacToeBoard.PLAYER_X) && 
									(player1.getPlayerType() == Player.HUMAN_PLAYER)) {
								((Human)player1).setChosenSquare(location.row, location.col);
								CustomPanel.this.chosen = true;
							}
							else if((turnState.getTurn() == TicTacToeBoard.PLAYER_O) && 
									(player2.getPlayerType() == Player.HUMAN_PLAYER)) {
								((Human)player2).setChosenSquare(location.row, location.col);
								CustomPanel.this.chosen = true;
							}
						}
					}
				}
			);
	}
	
	/**
	 * Resets the panel information.
	 * @param p1 Reference to player1's information.
	 * @param p2 Reference to player2's information.
	 */
	public void reset(Player p1, Player p2) {
		player1 = p1;
		player2 = p2;
		chosen = false;
	}
	
	/**
	 * Sets the chosen flag to the given value.
	 * @param value The value for the chosen flag.
	 */
	public void setChosen(boolean value) {
		this.chosen = value;
	}
}
