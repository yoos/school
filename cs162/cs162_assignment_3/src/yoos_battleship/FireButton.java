
package yoos_battleship;

import javax.swing.JButton;


public class FireButton extends JButton{
	private Cell cell;	
	
	public FireButton(int col, int row){
		cell = new Cell(col,row);
	}
	
	/**
	 * 
	 * @return the cell the button is at
	 */
	public Cell getCell(){
		return this.cell;
	}

}
