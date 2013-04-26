import java.util.*;
import static java.lang.Math.*;

/**
 * This class represents the module for minimax.
 * @author Chris Ventura
 *
 */
public class MiniMax implements Player {
	Position nextPos = new Position();

	/**
	 * Constructor
	 *
	 */
	public MiniMax() {

	}

	/**
	 * Returns the next move.
	 * @param state The current board state in the game
	 * @return The next move
	 */
	public Position getNextMove(TicTacToeBoard state) throws Exception {
		nextPos = minimaxDecision(state);

		return nextPos;
	}

	/**
	 * Returns the player type 
	 */
	public int getPlayerType() {
		return MINIMAX_PLAYER;
	}

	private List<TicTacToeBoard> successors(TicTacToeBoard state) {
		List<TicTacToeBoard> succ = new ArrayList<TicTacToeBoard>();

		// Whose turn is it?
		int turn = state.getTurn();

		// Generate successors and append to succ.
		for (int i=0; i<3; i++) {
			for (int j=0; j<3; j++) {
				if (state.getState(i, j) == TicTacToeBoard.BLANK) {
					// Make copy of game state.
					TicTacToeBoard s = new TicTacToeBoard();
					s = (TicTacToeBoard) state.clone();

					// Play.
					try {
						s.setState(i, j, turn);
					}
					catch (Exception e) {
						// Do nothing, because I'm too good for exceptions.
					}

					// Set next player.
					if (turn == TicTacToeBoard.PLAYER_X) {
						s.setTurn(TicTacToeBoard.PLAYER_O);
					}
					else {
						s.setTurn(TicTacToeBoard.PLAYER_X);
					}

					// Append to list of successors.
					succ.add(s);
				}
			}
		}

		return succ;
	}

	private int utility(TicTacToeBoard state) {
		int u = 0;
		try {
			if (state.isWin(TicTacToeBoard.PLAYER_X)) {
				u = 1;
			}
			else if (state.isWin(TicTacToeBoard.PLAYER_O)) {
				u = -1;
			}
		}
		catch (Exception e) {
			// Do nothing.
		}

		return u;
	}

	private boolean terminalTest(TicTacToeBoard state) {
		boolean testRes = false;
		try {
			testRes = state.isGameOver();
		}
		catch (Exception e) {
			// Do nothing.
		}

		return testRes;
	}

	private Position minimaxDecision(TicTacToeBoard state) {
		Position bestMove = new Position();

		int turn = state.getTurn();
		int v;
		if (turn == TicTacToeBoard.PLAYER_X) {
			v = maxValue(state);
		}
		else {
			v = minValue(state);
		}

		for (TicTacToeBoard s : successors(state)) {
			int sVal;
			if (turn == TicTacToeBoard.PLAYER_X) {
				sVal = minValue(s);
			}
			else {
				sVal = maxValue(s);
			}

			if (sVal == v) {
				bestMove = s.getLastMove();
			}
		}

		return bestMove;
	}

	private int maxValue(TicTacToeBoard state) {
		if (terminalTest(state)) {
			return utility(state);
		}

		int v = -1000000;

		for (TicTacToeBoard s : successors(state)) {
			v = max(v, minValue(s));
		}

		return v;
	}

	private int minValue(TicTacToeBoard state) {
		if (terminalTest(state)) {
			return utility(state);
		}

		int v = 1000000;

		for (TicTacToeBoard s : successors(state)) {
			v = min(v, maxValue(s));
		}

		return v;
	}
}

