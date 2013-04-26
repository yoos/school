/*! \file BattleshipGUI.java
 *  \author Soo-Hyun Yoo
 *  \brief Battleship GUI.
 */

package yoos_battleship;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.border.*;

import yoos_battleship.BattleshipBoard;
import yoos_battleship.FireButton;

public class BattleshipGUI extends JFrame {
    BattleshipBoard playerBoard, opponentBoard;
    Player player = new HumanPlayer();
    Player opponent;   // The opponent will be instantiated once we read in command line arguments.

    Container labelBox      = new Container();   // HBox to contain board labels.
    JTextPane playerLabel   = new JTextPane();   // Inside labelBox.
    JTextPane opponentLabel = new JTextPane();   // Inside labelBox.
    Container panelBox      = new Container();   // HBox to contain boards.
    JPanel playerPanel      = new JPanel(new GridLayout(10, 10));   // Inside panelBox.
    JPanel opponentPanel    = new JPanel(new GridLayout(10, 10));   // Inside panelBox.
    JTextPane winnerAnnouncePane = new JTextPane();   // Below panelBox, announces winner.

    Border blackline = BorderFactory.createLineBorder(Color.black);   // Black border for FireButtons

    /*! BattleshipGUI constructor.
     *
     *  \param playerBoard Text file for player board.
     *  \param opponentBoard Text file for opponent board.
     *  \param oppType 'r' or 's' to signify random or systematic player, respectively.
     */
    public BattleshipGUI(BattleshipBoard playerBoard,     // Text file
                         BattleshipBoard opponentBoard,   // Text file
                         char oppType) {                  // 'r' or 's'
        setTitle("Battleship");
        setSize(200, 200);   // Arbitrary.
        setLayout(new BoxLayout(getContentPane(), BoxLayout.Y_AXIS));

        System.out.println("Starting GUI!");

        // Load boards.
        this.playerBoard   = playerBoard;
        this.opponentBoard = opponentBoard;

        // Set labels and instantiate opponent player.
        playerLabel.setText("Human Player Board");
        playerLabel.setEditable(false);

        if (oppType == 'r') {
            opponent = new RandomPlayer();
            opponentLabel.setText("Random Player Board");
        }
        else if (oppType == 's') {
            opponent = new SystematicPlayer();
            opponentLabel.setText("Systematic Player Board");
        }
        opponentLabel.setEditable(false);

        // Fill up boxes.
        labelBox.setLayout(new BoxLayout(labelBox, BoxLayout.X_AXIS));
        labelBox.add(playerLabel);
        labelBox.add(opponentLabel);
        this.add(labelBox);

        this.add(Box.createRigidArea(new Dimension(780, 0)));   // Make window at least 600 pixels wide.
        panelBox.setLayout(new BoxLayout(panelBox, BoxLayout.X_AXIS));
        panelBox.add(playerPanel);
        playerPanel.setPreferredSize(new Dimension(350, 350));
        panelBox.add(Box.createRigidArea(new Dimension(5, 300)));   // Make panelBox at least 300 pixels high with 5 pixels in between the two JPanels.
        panelBox.add(opponentPanel);
        opponentPanel.setPreferredSize(new Dimension(350, 350));
        this.add(panelBox);

        winnerAnnouncePane.setEditable(false);
        this.add(winnerAnnouncePane);

        // Fill up panels with FireButtons. Note that Swing seems to fill the panel top to bottom, THEN left to right (NOT the other way around).
        for (int i=0; i<10; i++) {       // Rows
            for (int j=0; j<10; j++) {   // Columns
                try {
                    // Disabled FireButtons for playerPanel.
                    FireButton fb1 = new FireButton(j, i);
                    fb1.setEnabled(false);
                    fb1.setBorder(blackline);

                    // Color cells appropriately based on player's ship placement.
                    if (playerBoard.getCellContent(j, i) != BattleshipBoard.EMPTY) {
                        fb1.setBackground(Color.GRAY);
                        fb1.setText(playerBoard.getCellContent(j, i) + "");
                    }
                    else {
                        fb1.setBackground(Color.BLUE);
                    }

                    playerPanel.add(fb1);

                    // FireButtons for opponentPanel.
                    FireButton fb2 = new FireButton(j, i);
                    fb2.setBorder(blackline);
                    fb2.setBackground(Color.BLUE);
                    fb2.addActionListener(new UpdateCellListener());

                    opponentPanel.add(fb2);
                }
                catch (Exception e) {
                    System.out.println(e.getMessage());
                    System.exit(-1);
                }
            }
        }

        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        pack();
        setVisible(true);
    }

    /*! Disable all buttons.
     */
    private void gameOver() {
        for (Component each : opponentPanel.getComponents()) {
            if (each instanceof FireButton) {
                each.setEnabled(false);
            }
        }
    }

    /*! React to a pressed FireButton.
     */
    class UpdateCellListener implements ActionListener {
        public void actionPerformed (ActionEvent e) {
            if (e.getSource() instanceof FireButton) {
                // Grab the pointer to the button pressed.
                FireButton fbOpp = (FireButton) e.getSource();

                // Get the button location.
                int col = fbOpp.getCell().getColumn();
                int row = fbOpp.getCell().getRow();

                // Human player goes first.
                int playerResult   = player.play(opponentBoard, col, row);

                // Redraw opponent board.
                if (playerResult == 1) {
                    fbOpp.setText("H");
                    fbOpp.setBackground(Color.RED);
                }
                else {
                    fbOpp.setText("M");
                    fbOpp.setBackground(Color.BLACK);
                }
                fbOpp.setEnabled(false);

                // Is game over?
                if (opponentBoard.isGameOver()) {
                    winnerAnnouncePane.setText(playerLabel.getText() + " wins!");
                    gameOver();
                    return;
                }

                // Computer goes second.
                int opponentResult = opponent.play(playerBoard);

                // Get the location of opponent's last target.
                FireButton fbPlr = (FireButton) playerPanel.getComponents()[10*opponent.getPrevTargetRow()+opponent.getPrevTargetCol()];

                // Redraw player board.
                if (opponentResult == 1) {
                    fbPlr.setText("H");
                    fbPlr.setBackground(Color.RED);
                }
                else {
                    fbPlr.setText("M");
                    fbPlr.setBackground(Color.BLACK);
                }

                // Is game over?
                if (playerBoard.isGameOver()) {
                    winnerAnnouncePane.setText(opponentLabel.getText() + " wins!");
                    gameOver();
                }
            }
        }
    }
}

