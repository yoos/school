import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.JTextField;

public class ChatBox extends JFrame {
	JTextArea    myTextArea   = new JTextArea();
	JTextField   myTextField  = new JTextField(35);
	AddTextListener addLisr   = new AddTextListener();
	JScrollPane  myScrollPane = new JScrollPane(myTextArea);
	JPanel       myPanel      = new JPanel();
	ClearTextListener clrLisr = new ClearTextListener();
	JButton      myButton     = new JButton("Clear");

	public ChatBox () {
		setTitle("Simple chat box");
		setSize(500, 300);
		
		this.add(myScrollPane);
		myTextArea.setEditable(false);
		
		this.add(myPanel, BorderLayout.SOUTH);
		myPanel.add(myTextField, BorderLayout.WEST);
		myPanel.add(myButton, BorderLayout.EAST);
		
		myTextField.addActionListener(addLisr);
		myButton.addActionListener(clrLisr);
	}
	
	class AddTextListener implements ActionListener {
		public void actionPerformed (ActionEvent event) {
			myTextArea.append(myTextField.getText() + "\n");
			myTextField.setText(null);
		}
	}

	class ClearTextListener implements ActionListener {
		public void actionPerformed (ActionEvent event) {
			myTextArea.setText(null);
		}
	}
	
	public static void main (String [] args) {
		JFrame box = new ChatBox();
		
		box.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		box.setVisible(true);
	}
}
