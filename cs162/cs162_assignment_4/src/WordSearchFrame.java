import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.StringTokenizer;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.border.EtchedBorder;
import javax.swing.border.TitledBorder;

/**
 * This class represents a word search puzzle frame. It displays the word search
 * along with the list of words to find in the word search.
 * 
 * @author wong
 * 
 */
public class WordSearchFrame extends JFrame {

	// Leave the line below in your code. You won't need to know what it is for
	// this course.
	private static final long serialVersionUID = 1L;

	// The number of columns in the word list
	private static final int COLS_IN_WORDLIST = 1;

	// The width of the frame
	private static final int FRAME_WIDTH = 640;

	// The height of the frame
	private static final int FRAME_HEIGHT = 480;

	// The grid used in the word search
	private JLabel[][] wordSearch;

	// The labels used to display the words in the word list
	private JLabel[] wordListLabels;

	// The panel that the word list is inside
	private JPanel wordListPanel;

	// The panel that the word search is inside
	private JPanel wordsearchPanel;

	// The panel on the right hand side
	private JPanel rightSidePanel;

	// The panel on the left hand side
	private JPanel leftSidePanel;

	// The panel that the search button is inside
	private JPanel searchButtonPanel;

	// The search button
	private JButton searchButton;

	// The number of rows in the word search
	private int numRows;

	// The number of columns in the word search
	private int numCols;

	// The listener for the search button
	private ActionListener searchButtonListener;

	// The list of words to be found in the word search
	// If this line won't compile in Java 1.4.2, change it to
	// private ArrayList wordList;
	private ArrayList<String> wordList;


    private int currentWordIndex;   // Keep track of which word we're searching for.

    private static final int[][] dir = {{ 1,  0},    // E
                                        { 1, -1},    // NE
                                        { 0, -1},    // N
                                        {-1, -1},    // NW
                                        {-1,  0},    // W
                                        {-1,  1},    // SW
                                        { 0,  1},    // S
                                        { 1,  1}};   // SE

    /*! Recursive word finder.
     *
     *  \param word Word to find.
     *  \param col Search start column.
     *  \param row Search start row.
     *  \param colInc Column increment number given by dir[][].
     *  \param rowInc Row increment number given by dir[][].
     *
     *  \return Ending coordinates of word if any are found, (-1, -1) if not.
     */
    private int[] findWord(String word, int col, int row, int colInc, int rowInc) {
        int len = word.length();
        int[] ret = {-1, 1};

        // Don't search past edges of wordsearch.
        if (col < 0 || row < 0 || col > numCols-1 || row > numRows-1) {
            // Do nothing.
        }
        else if (wordSearch[col][row].getText().charAt(0) == word.charAt(0)) {
            // Base case.
            if (len == 1) {
                ret[0] = col;
                ret[1] = row;
            }
            // Recursive step.
            else {
                int[] tmp = findWord(word.substring(1, len), col+colInc, row+rowInc, colInc, rowInc);
                ret[0] = tmp[0];
                ret[1] = tmp[1];
            }
        }

        return ret;
    }

    /*! Word colorer.
     *
     *  \param col Start column.
     *  \param row Start row.
     *  \param dir Direction in which to color.
     *  \param len The length of the word to color.
     */
    private void colorWord(int col, int row, int[] dir, int len) {
        // Wipe matrix clean.
        for (int i=0; i<numCols; i++) {
            for (int j=0; j<numRows; j++) {
                wordSearch[i][j].setForeground(Color.BLACK);
            }
        }

        // Color cells.
        for (int i=0; i<len; i++) {
            wordSearch[col + i*dir[0]][row + i*dir[1]].setForeground(Color.RED);
        }
    }

    /**
     * The listener for the search button
     */
    class SearchListener implements ActionListener {
        /**
         * This event handler causes the current word in the word list to be
         * colored in red and finds the word in the word search
         * 
         * @param event
         *            The event corresponding to the search button being clicked
         */
        public void actionPerformed(ActionEvent event) {
            int[] loc = {-1, -1};
            int mod = wordList.size();

            for (int i=0; i<numCols; i++) {
                for (int j=0; j<numRows; j++) {
                    for (int k=0; k<8; k++) {
                        // Find the word.
                        loc = findWord(wordList.get(currentWordIndex), i, j, dir[k][0], dir[k][1]);

                        // If a word was found, loc[0] and loc[1] should both be non-negative.
                        if (loc[0] >= 0) {
                            System.out.println(wordList.get(currentWordIndex) + " found between (" + i + ", " + j + ") and (" + loc[0] + ", " + loc[1] + ") !");

                            colorWord(i, j, dir[k], wordList.get(currentWordIndex).length());   // Color word.
                            wordListLabels[currentWordIndex].setForeground(Color.RED);   // Color current word label.
                            wordListLabels[(currentWordIndex+mod-1) % mod].setForeground(Color.BLUE);   // Uncolor previous word label.
                            currentWordIndex = (currentWordIndex + 1) % mod;   // Increment word index and take mod.

                            return;
                        }
                    }
                }
            }
        }
    }

	/**
	 * Builds the panel on the left hand side, which consists of the word search
	 * 
	 * @throws WordSearchException
	 *             Throws a WordSearchException if the word search file isn't in
	 *             the proper format
	 * @throws IOException
	 *             Throws an IOException if loading the word search file has
	 *             problems
	 */
	private void buildLeftSidePanel() throws WordSearchException, IOException {
		// Create leftSidePanel
		leftSidePanel = new JPanel();
		leftSidePanel.setLayout(new BorderLayout());
		leftSidePanel.setBorder(new TitledBorder(new EtchedBorder(),
				"Word Search"));

		// Build wordsearch panel
		wordsearchPanel = new JPanel();
		wordsearchPanel.setLayout(new GridLayout(numRows, numCols));
		leftSidePanel.add(wordsearchPanel, BorderLayout.CENTER);

		// Add leftSidePanel panel to main frame
		this.getContentPane().add(leftSidePanel, BorderLayout.CENTER);
	}

	/**
	 * Determines the number of columns and rows in the word search file.
	 * 
	 * @param wordSearchFilename
	 *            The file name of the word search file
	 * @throws WordSearchException
	 *             Throws a WordSearchException if the word search file isn't in
	 *             the proper format
	 * @throws IOException
	 *             Throws an IOException if loading the word search file has
	 *             problems
	 */
	private void initGridFromFile(String wordSearchFilename)
			throws WordSearchException, IOException {
		this.numRows = 0;
		this.numCols = 0;
		BufferedReader reader = new BufferedReader(new FileReader(
				wordSearchFilename));
		String line = reader.readLine();
		while (line != null) {
			StringTokenizer tokenizer = new StringTokenizer(line, " ");
			if (this.numCols == 0) {
				this.numCols = tokenizer.countTokens();
			} else {
				if (tokenizer.countTokens() != this.numCols) {
					throw new WordSearchException(
							"Invalid number of columns in word search");
				}
			}
			line = reader.readLine();
			this.numRows++;
		}
		reader.close();
		this.wordSearch = new JLabel[numRows][numCols];
	}

	/**
	 * Loads the contents of the word search into the JLabels of the word search
	 * grid in the GUI
	 * 
	 * @param wordSearchfilename
	 *            The name of the word search file
	 * @throws IOException
	 *             Throws an IOException if loading the word search file has
	 *             problems
	 */
	protected void loadGridFromFile(String wordSearchFilename)
			throws IOException {
		int row = 0;
		BufferedReader reader = new BufferedReader(new FileReader(
				wordSearchFilename));
		String line = reader.readLine();
		while (line != null) {
			StringTokenizer tokenizer = new StringTokenizer(line, " ");
			int col = 0;
			while (tokenizer.hasMoreTokens()) {
				String tok = tokenizer.nextToken();
				wordSearch[row][col] = new JLabel(tok);
				wordSearch[row][col].setForeground(Color.BLACK);
				wordSearch[row][col]
						.setHorizontalAlignment(SwingConstants.CENTER);
				wordsearchPanel.add(wordSearch[row][col]);
				col++;
			}
			line = reader.readLine();
			row++;
		}
		reader.close();
	}

	/**
	 * Builds the right hand side panel which contains the list of words in the
	 * word search and the search button
	 */
	private void buildRightSidePanel() {
		// Build the panel on the right side
		rightSidePanel = new JPanel();
		rightSidePanel.setBorder(new TitledBorder(new EtchedBorder(),
				"Word List"));
		rightSidePanel.setLayout(new BorderLayout());

		// Build the word list
		wordListLabels = new JLabel[wordList.size()];
		wordListPanel = new JPanel();
		wordListPanel.setLayout(new GridLayout(wordList.size(), 1));
		for (int i = 0; i < this.wordList.size(); i++) {
			// If the line below won't compile in Java 1.4.2, make it
			// String word = (String)this.wordList.get(i);
			String word = this.wordList.get(i);
			wordListLabels[i] = new JLabel(word);
			wordListLabels[i].setForeground(Color.BLUE);
			wordListLabels[i].setHorizontalAlignment(SwingConstants.CENTER);
			wordListPanel.add(wordListLabels[i]);
		}
		rightSidePanel.add(wordListPanel, BorderLayout.CENTER);

		// Add search button
		searchButton = new JButton("Search");
		searchButtonListener = new SearchListener();
		searchButton.addActionListener(searchButtonListener);
		searchButtonPanel = new JPanel();
		searchButtonPanel.add(searchButton);
		rightSidePanel.add(searchButtonPanel, BorderLayout.SOUTH);

		// Add rightSidePanel to main panel
		this.getContentPane().add(rightSidePanel, BorderLayout.EAST);
	}

	/**
	 * Loads the list of words to find in the word search
	 * 
	 * @param wordListFilename
	 *            The file name of the file containing the list of words
	 * @throws WordSearchException
	 *             Throws a WordSearchException if the word search file isn't in
	 *             the proper format
	 * @throws IOException
	 *             Throws an IOException if loading the word search file has
	 *             problems
	 */
	private void loadWordList(String wordListFilename)
			throws WordSearchException, IOException {
		int row = 0;

		// If this line won't compile under Java 1.4.2 change it to
		// this.wordList = new ArrayList();
		wordList = new ArrayList<String>();
		BufferedReader reader = new BufferedReader(new FileReader(
				wordListFilename));
		String line = reader.readLine();
		while (line != null) {
			StringTokenizer tokenizer = new StringTokenizer(line, " ");
			if (tokenizer.countTokens() != COLS_IN_WORDLIST) {
				throw new WordSearchException(
						"Error: only one word per line allowed in the word list");
			}
			String tok = tokenizer.nextToken();
			// If this line won't compile, change it to
			// wordList.add((Object)tok)
			wordList.add(tok);
			line = reader.readLine();
			row++;
		}
		reader.close();
	}

	/**
	 * Constructor for the WordSearchFrame
	 * 
	 * @param wordSearchFilename
	 *            The name of the word search file
	 * @param wordListFilename
	 *            The name of the word list file
	 * @throws IOException
	 *             Throws an IOException if loading the word search file has
	 *             problems
	 * @throws WordSearchException
	 *             Throws a WordSearchException if the word search file isn't in
	 *             the proper format
	 */
	public WordSearchFrame(String wordSearchFilename, String wordListFilename)
			throws IOException, WordSearchException {
		this.setSize(FRAME_WIDTH, FRAME_HEIGHT);
		this.getContentPane().setLayout(new BorderLayout());
		this.initGridFromFile(wordSearchFilename);
		buildLeftSidePanel();
		this.loadGridFromFile(wordSearchFilename);
		loadWordList(wordListFilename);
		buildRightSidePanel();

        currentWordIndex = 0;
	}

	/**
	 * Constructor which takes a wordSearch 2D String array and a wordList String array
	 * @param wordSearch The word search represented as a 2D String array
	 * @param wordList The list of words to search for
	 * @throws IOException Throws an IOException if loading the word search file has
	 *             problems
	 * @throws WordSearchException  Throws a WordSearchException if the word search file isn't in
	 *             the proper format
	 */
	public WordSearchFrame(String[][] wordSearch, String[] wordList)
			throws IOException, WordSearchException {
		this.setSize(FRAME_WIDTH, FRAME_HEIGHT);
		this.getContentPane().setLayout(new BorderLayout());
		this.numRows = wordSearch.length;
		this.numCols = wordSearch[0].length;
		this.wordSearch = new JLabel[this.numRows][this.numCols];
		buildLeftSidePanel();
		for( int row = 0; row < this.numRows; row++ ) {
			for( int col = 0; col < this.numCols; col++ ) {
				this.wordSearch[row][col] = new JLabel(wordSearch[row][col]);
				this.wordSearch[row][col].setForeground(Color.BLACK);
				this.wordSearch[row][col]
						.setHorizontalAlignment(SwingConstants.CENTER);
				this.wordsearchPanel.add(this.wordSearch[row][col]);
			}
		}

		
		// If this line won't compile under Java 1.4.2 change it to
		// this.wordList = new ArrayList();
		this.wordList = new ArrayList<String>();
		for( int i = 0; i < wordList.length; i++ ) {
			// If this line won't compile, change it to
			// this.wordList.add((Object)wordList[i])
			this.wordList.add(wordList[i]);
		}
		buildRightSidePanel();
	}

	/**
	 * @param args
	 *            The first argument is the word search file, the second
	 *            argument is the word list.
	 */
	public static void main(String[] args) {
		try {
			if (args.length != 2) {
				System.out
						.println("Command line arguments: <word search file> <word list>");
			} else {
				WordSearchFrame frame = new WordSearchFrame(args[0], args[1]);
				frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
				frame.setVisible(true);
			}
		} catch (Exception e) {
			System.out.println(e.getMessage());
			e.printStackTrace();
		}
	}
}
