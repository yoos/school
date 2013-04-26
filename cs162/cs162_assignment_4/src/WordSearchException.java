/**
 * This file represents a word search exception class.
 * @author wong
 *
 */
public class WordSearchException extends RuntimeException {
	// Leave the line below in your code. You won't need to know what it is
	// for this course.
	private static final long serialVersionUID = 1L;

	/**
	 * Default constructor
	 */
	public WordSearchException() {

	}

	/**
	 * Constructor with an error message
	 * 
	 * @param reason
	 *            The error message for the exception
	 */
	public WordSearchException(String reason) {
		super(reason);
	}
}
