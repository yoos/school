/**
 * MessageScrambler class that takes a message, encrypts it using a key, and
 * then allows one to decrypt it.
 * This is used in CS 162 Lab 1
 * @author Wong
 *
 */
public class MessageScrambler {
	
	/** The key used for encryption/decryption */
	private String key;
	
	/** 
	 * Constructor for the MessageScrambler class
	 * @param key The key used for encryption/decryption
	 */
	public MessageScrambler(String key) {
		this.key = key;
	}
	
	/**
	 * Initializes the temporary buffer used for encryption/decryption.
	 * @param message The message to encrypt or decrypt 
	 * @return
	 */
	private StringBuffer initTempBuffer(String message) {
		StringBuffer tempBuffer = new StringBuffer(message);
		for( int i = 0, j = 0; i < message.length(); i++ ) {
			tempBuffer.setCharAt(i,this.key.charAt(j));
			j = (j+1) % key.length();
		}	
		/****************************************
		 * Breakpoint #2 goes on the line below *
		 ****************************************/
		return tempBuffer;
	}

	/**
	 * Encrypts the message
	 * @param message The message you want to encrypt
	 * @return The encrypted message
	 */
	public String encrypt(String message) {
		StringBuffer result = initTempBuffer(message);
		for( int i = 0; i < message.length(); i++ ) {
			
			// If you are curious what this is doing...
			// This is a simple cryptography trick...
			// The ^ operator performs an exclusive OR.
			// The character at position i for result buffer is first converted 
			// to an int. Then the character at position i for the message is
			// also converted to an int. The two ints are exclusive ORed together
			// and the result is converted to a char and put back into the result
			// buffer at position i.
			char encryptedChar = (char)((int)result.charAt(i) ^ (int)message.charAt(i));
			result.setCharAt(i,encryptedChar);
		}
		return result.toString();
	}
	
	/**
	 * Decrypts the encrypted message
	 * @param message The message you wanted to decrypt
	 * @return The decrypted message
	 */
	public String decrypt(String message) {
		StringBuffer result = initTempBuffer(message);
		for( int i = 0; i < message.length(); i++ ) {
			
			// The ^ operator performs an exclusive OR
			char decryptedChar = (char)((int)result.charAt(i) ^ (int)message.charAt(i));
			result.setCharAt(i,decryptedChar);
		}
		return result.toString();
	}
	
	/**
	 * The main function
	 * @param args Command line arguments
	 */
	public static void main(String args[]) {
		/****************************************
		 * Breakpoint #1 goes on the line below *
		 ****************************************/
		String message = "Philomath";
		System.out.println("Original message: " + message);
		MessageScrambler ms = new MessageScrambler("java");
		String encryptedMessage = ms.encrypt(message);
		String decryptedMessage = ms.decrypt(encryptedMessage);
		System.out.println("Done");
	}
}
