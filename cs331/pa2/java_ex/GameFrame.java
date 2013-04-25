package edu.oregonstate.eecs.cs331.assn2;

import java.awt.*;
import javax.swing.*;

/**
 * The GameFrame class is the only frame/window of the GUI.
 * 
 * @author Chris Ventura
 * 
 */
public class GameFrame extends JFrame {
	
	/**
	 * Command line strings that specify the types of the players
	 */
	private final static String HUMAN_PLAYER_STRING = "human";
	private final static String RANDOM_PLAYER_STRING = "random";
	private final static String MINIMAX_PLAYER_STRING = "minimax";

	/**
	 * This is to keep the compiler happy.
	 */
	private static final long serialVersionUID = 1L;

	/**
	 * The JPanel that contains the TicTacToe board.
	 */
	private JPanel gameGrid;

	/**
	 * The customized JPanels that define the layout of the game board.
	 */
	private CustomPanel cellGrid[];

	/**
	 * The labels used the interface specifying whose turn it is and which
	 * squares have been taken by a player.
	 */
	private JLabel turnLabel, cellLabel[];

	/**
	 * The strings to be used with the turnLabel, specifying whose turn it is.
	 */
	private String turnString[];

	/**
	 * Reference to the class containing the game's state information.
	 */
	private TicTacToeBoard boardState;

	/**
	 * These two references are used to interact with the appropriate player.
	 */
	private Player playerX;
	private Player playerO;

	/**
	 * This is the content pane for the main JFrame
	 */
	Container contentPane;
	/**
	 * Builds the Game board GUI
	 *
	 */
	private void buildGameBoard() {
		gameGrid = new JPanel(new GridLayout(TicTacToeBoard.SIZE,
				TicTacToeBoard.SIZE));
		cellGrid = new CustomPanel[TicTacToeBoard.SIZE * TicTacToeBoard.SIZE];
		cellLabel = new JLabel[TicTacToeBoard.SIZE * TicTacToeBoard.SIZE];
		for (int count = 0; count < cellGrid.length; count++) {
			cellGrid[count] = new CustomPanel(count / TicTacToeBoard.SIZE,
					count % TicTacToeBoard.SIZE, playerX, playerO, boardState);
			cellLabel[count] = new JLabel("");
			cellLabel[count].setHorizontalAlignment(SwingConstants.CENTER);
			cellLabel[count].setVerticalAlignment(SwingConstants.CENTER);
			cellGrid[count].setLayout(new BorderLayout());
			cellGrid[count].add(cellLabel[count], BorderLayout.CENTER);
			gameGrid.add(cellGrid[count]);
		}
		for (int cellCount = 0; cellCount < cellGrid.length; cellCount++) {
			switch (cellCount) {
			case 0:
			case 1:
			case 3:
			case 4:
				cellGrid[cellCount].add(new JSeparator(
						SwingConstants.HORIZONTAL), BorderLayout.SOUTH);
				cellGrid[cellCount].add(
						new JSeparator(SwingConstants.VERTICAL),
						BorderLayout.EAST);
				break;
			case 2:
			case 5:
				cellGrid[cellCount].add(new JSeparator(
						SwingConstants.HORIZONTAL), BorderLayout.SOUTH);
				break;
			case 6:
			case 7:
				cellGrid[cellCount].add(
						new JSeparator(SwingConstants.VERTICAL),
						BorderLayout.EAST);
				break;
			default:
				break;
			}
		}
		contentPane.add(gameGrid);

	}

	/**
	 * Builds the turn label which states which player can go next.
	 *
	 */
	private void buildTurnLabel() {
		boardState.setTurn(TicTacToeBoard.PLAYER_X);
		turnLabel = new JLabel();
		turnLabel.setText(turnString[boardState.getTurn()]);
		turnLabel.setVerticalTextPosition(SwingConstants.CENTER);
		turnLabel.setHorizontalTextPosition(SwingConstants.CENTER);
		turnLabel.setHorizontalAlignment(SwingConstants.CENTER);
		turnLabel.setVerticalAlignment(SwingConstants.CENTER);
		contentPane.add(turnLabel, BorderLayout.SOUTH);
	}


	/**
	 * This constructor builds and places of all of the objects on the Game
	 * Frame.
	 */
	public GameFrame(int playerType1, int playerType2) throws Exception {
		super("TicTacToe");
		boardState = new TicTacToeBoard();
		turnString = new String[2];
		turnString[0] = "X's Turn";
		turnString[1] = "O's Turn";

		// The 10's specify the amount of whitespace
		contentPane = this.getContentPane();
		contentPane.setLayout(new BorderLayout(10, 10));
		buildGameBoard();
		buildTurnLabel();
		// A fixed size so that the form looks clean
		setSize(200, 200);
		setVisible(true);
		setResizable(false);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		newGame(playerType1, playerType2);
	}

	/**
	 * Used to create a new game when "New" is selected from the "File" menu.
	 * 
	 * @param type1
	 *            Player1's type.
	 * @param type2
	 *            Player2's type.
	 */
	private void newGame(int type1, int type2) {
		switch (type1) {
		case Player.RANDOM_PLAYER:
			playerX = new RandomPlayer();
			break;
		case Player.MINIMAX_PLAYER:
			playerX = new MiniMax();
			break;
		default:
			playerX = new Human();
			break;
		}

		switch (type2) {
		case Player.RANDOM_PLAYER:
			playerO = new RandomPlayer();
			break;
		case Player.MINIMAX_PLAYER:
			playerO = new MiniMax();
			break;
		default:
			playerO = new Human();
			break;
		}
		for (int count = 0; count < cellGrid.length; count++) {
			cellGrid[count].reset(playerX, playerO);
			cellLabel[count].setText("");
		}
		boardState.setTurn(TicTacToeBoard.PLAYER_X);
		turnLabel.setText(turnString[TicTacToeBoard.PLAYER_X]);
	}

	/**
	 * Returns true if the position is a legal one.  A legal position is one in which
	 * the row takes a value of 0, 1, or 2 and the column takes the value of 0, 1, or 2.
	 * @param p The position you want to check the legality of
	 * @return True if the position is legal, false otherwise
	 */
	private boolean isLegalPosition(Position p) {
		if ((p != null) && (p.row >= 0) && (p.row < TicTacToeBoard.SIZE)
				&& (p.col >= 0) && (p.col < TicTacToeBoard.SIZE)) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * This is where the logic of the game is actually carried out.
	 * @throws Exception 
	 */
	public void playGame() throws Exception {
		Position updatedCell;
		while (!boardState.isGameOver()) {
			if (boardState.getTurn() == TicTacToeBoard.PLAYER_X) {
				updatedCell = playerX.getNextMove(boardState);
				if (isLegalPosition(updatedCell)) {
					boardState.setState(updatedCell.row, updatedCell.col,
							TicTacToeBoard.X);
					cellGrid[updatedCell.row*TicTacToeBoard.SIZE+updatedCell.col].setChosen(true);
				} else {
					throw new Exception("Illegal board position returned");
				}
				boardState.setTurn(TicTacToeBoard.PLAYER_O);
			} else {
				updatedCell = playerO.getNextMove(boardState);
				if (isLegalPosition(updatedCell)) {
					boardState.setState(updatedCell.row, updatedCell.col,
							TicTacToeBoard.O);
					cellGrid[updatedCell.row*TicTacToeBoard.SIZE+updatedCell.col].setChosen(true);
				} else {
					throw new Exception("Illegal board position returned");
				}
				boardState.setTurn(TicTacToeBoard.PLAYER_X);
			}

			displayBoard(boardState);
			turnLabel.setText(turnString[boardState.getTurn()]);
		}

		if (boardState.isWin(TicTacToeBoard.PLAYER_X)) {
			turnLabel.setText("Player X won");
		} else if (boardState.isWin(TicTacToeBoard.PLAYER_O)) {
			turnLabel.setText("Player O won");
		} else {
			// Draw
			turnLabel.setText("Draw");
		}
	}

	/**
	 * Updates the labels in the main Frame to reflect the specified state of
	 * the game.
	 * 
	 * @param state
	 *            The state of the game.
	 */
	private void displayBoard(TicTacToeBoard state) {
		for (int row = 0; row < TicTacToeBoard.SIZE; row++) {
			for (int col = 0; col < TicTacToeBoard.SIZE; col++) {
				if (state.getState(row, col) == TicTacToeBoard.X) {
					cellLabel[row * TicTacToeBoard.SIZE + col].setText("X");
				} else if (state.getState(row, col) == TicTacToeBoard.O) {
					cellLabel[row * TicTacToeBoard.SIZE + col].setText("O");
				} else {
					cellLabel[row * TicTacToeBoard.SIZE + col].setText("");
				}
			}
		}
	}

	/**
	 * The entry point for this program.
	 * 
	 * @param args
	 *            The command line arguments (Not used!)
	 */
	public static void main(String[] args) {
		try {
			int player1Type = Player.HUMAN_PLAYER;
			int player2Type = Player.HUMAN_PLAYER;
			if( args.length != 2  ) {
				System.out.println("Usage: program <player 1 type> <player 2 type>");
				System.out.println("       where player type = human, random, or minimax");
				System.exit(-1);
			}
			if( args[0].equals(HUMAN_PLAYER_STRING) ) {
				player1Type = Player.HUMAN_PLAYER;
			} else if( args[0].equals(RANDOM_PLAYER_STRING)) {
				player1Type = Player.RANDOM_PLAYER;
			} else if( args[0].equals(MINIMAX_PLAYER_STRING)) {
				player1Type = Player.MINIMAX_PLAYER;
			} else {
				throw new Exception("Unrecognized player type");
			}
			
			if( args[1].equals(HUMAN_PLAYER_STRING) ) {
				player2Type = Player.HUMAN_PLAYER;
			} else if( args[1].equals(RANDOM_PLAYER_STRING)) {
				player2Type = Player.RANDOM_PLAYER;
			} else if( args[1].equals(MINIMAX_PLAYER_STRING)) {
				player2Type = Player.MINIMAX_PLAYER;
			} else {
				throw new Exception("Unrecognized player type");
			}

			GameFrame application = new GameFrame(player1Type,
					player2Type);
			application.playGame();
		} catch (Exception e) {
			System.out.println(e.getMessage());
			e.printStackTrace();
		}
	}

}
