import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;


public class ImageViewer extends JFrame {
	
	ImagePanel panel = new ImagePanel();
	JButton button = new JButton("Open");
	OpenFileListener listener = new OpenFileListener();
	JFileChooser chooser = new JFileChooser();

	public ImageViewer() {
		setTitle("CS162 image viewer");
		setSize(400, 300);
		
		this.add(panel);
		this.add(button, BorderLayout.SOUTH);
		button.addActionListener(listener);
		chooser.setCurrentDirectory(null);
	}
	
	class OpenFileListener implements ActionListener {
		public void actionPerformed(ActionEvent event) {
			chooser.showDialog(chooser, "Open");
			String myFileName = chooser.getSelectedFile().getPath();
			panel.setImage(myFileName);
			System.out.println("pressed");
		}
	}

	public static void main (String [] args) {
		JFrame gui = new ImageViewer();
		
		gui.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		gui.setVisible(true);
	}
}
