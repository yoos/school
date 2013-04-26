/**
 * This class is a buggy version of the extreme value finder used in CS 162 Lab 2
 * @author wong
 *
 */
public class BuggyExtremeValueFinder {

	/** Values that you want to find the min and max for */
	private double[] v;
	
	/**
	 * Default constructor. Copies the entries in the vals array to the
	 * private member variable array v.
	 */
	public BuggyExtremeValueFinder(double vals[]) {
		this.v = new double[vals.length];
		for( int i = 0; i < vals.length; i++ ) {
			this.v[i] = vals[i];
		}
	}
	
	/**
	 * Finds the extreme values (minimum and maximum values) in the vals array
	 * @param vals The array of values that you want to obtain the minimum and maximum values
	 * for.
	 */
	public void findExtremeValues() {
		double minSoFar = this.v[0];
		double maxSoFar = this.v[0];
		for( int i = 0; i < (this.v.length); i++ ) {
			if( this.v[i] < minSoFar )
				minSoFar = this.v[i];
			if( this.v[i] > maxSoFar ) 
				maxSoFar = this.v[i];
		}
		System.out.println("min = " + minSoFar);
		System.out.println("max = " + maxSoFar);
	}
	
	/**
	 * Main function to start the program
	 * @param args command line arguments. Nothing here for now.
	 */
	public static void main(String args[]) {
		/****************************************
		 * Breakpoint #1 goes on the line below *
		 ****************************************/
		double vals[] = {-4.0, -3.5, -1.0};
		BuggyExtremeValueFinder finder = new BuggyExtremeValueFinder(vals);
		finder.findExtremeValues();
	}
}
