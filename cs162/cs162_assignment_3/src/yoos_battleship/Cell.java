
package yoos_battleship;

public class Cell {
	private int row;
	private int col;
	
	public Cell(int col, int row){
		this.row = row;
		this.col = col;
	}
	
	public int getRow(){
		return this.row;
	}
	
	public int getColumn(){
		return this.col;
	}
}
