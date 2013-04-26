import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;


/**
 * This displays an image as a panel. The image will be resized to fit inside the JPanel.
 *  
 * @author Christoph Neumann
 */

public class ImagePanel extends JPanel {
	private Image img;
	private int width, height;

	/**
	 * An no-argument constructor for when we just need a placeholder panel.
	 */
	public ImagePanel() {
		super();
		setBackground(Color.black);
	}

	/**
	 * A constructor for when we want to start with the given image in the panel.
	 * @param filename
	 */
	public ImagePanel(String filename) {
		super();
		setBackground(Color.black);

		loadImage(filename);
		if (img != null) {
			// By default, we'll set the dimensions of the panel to match the
			// actual size of the image.
			setSize(width, height);
		}
	}

	/**
	 * This draws the image on the panel. The image will be scaled and
	 * vertically and horizontally centered.
	 */
	public void paintComponent(Graphics g) {
		// Do any panel-specific painting first
		super.paintComponent(g);

		// Make sure the scaling is smooth
		Graphics2D g2 = (Graphics2D) g;
		g2.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);

		if (img != null) {
			// Calculate the amount the width and the height must be scaled. The
			// smallest scale factor is the one that is used. Also, an offset
			// amount is calculated for the vertical and horizontal centering.
			double scale_width = (double) getWidth() / width;
			double scale_height = (double) getHeight() / height;
			double scale = scale_width;
			int x_offset = 0;
			int y_offset = (int) ((getHeight() - height * scale) / 2);
			if (scale_height < scale_width) {
				scale = scale_height;
				x_offset = (int) ((getWidth() - width * scale) / 2);
				y_offset = 0;
			}

			// Draw the scaled image.
			g.drawImage(img, x_offset, y_offset, (int) (width * scale), (int) (height * scale), null);
		}
	}

	/**
	 * This changes the image being displayed in the panel.
	 * @param filename The full path to the image file to display.
	 */
	public void setImage(String filename) {
		loadImage(filename);
		repaint();
	}

	private void loadImage(String filename) {
		// Allow the called to "blank out" the image.
		if ( filename.equals("") ) {
			img = null;
			width = 0;
			height = 0;
			return;
		}

		// Try to load the image file. For simplicity, exceptions are simply
		// logged and not passed back to the caller.
		File file = new File(filename);
		try {
			img = ImageIO.read(file);
			if ( img != null ) {
				width = img.getWidth(null);
				height = img.getHeight(null);
			} else {
				System.err.println("Error: Not an image file: "+ file.getPath());
			}
		} catch (IOException e) {
			System.err.println("Error loading image: "+ file.getPath() +" "+ e.getMessage());
			e.printStackTrace();
		}
	}
}