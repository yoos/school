/**
 * This class represents a Pirate!  Arrr matey!
 *
 */

public class Pirate {
	// The amount of attack damage a pirate attack causes
	final static private int ATTACK_DAMAGE = 10;
	
	// The default health value of a pirate
	final static private int DEFAULT_HEALTH = 100;

	// The name of the pirate
	private String name;
	
	// The health of the pirate, represented as a numeric value
	private int health;
	
	// The amount of gold in the pirate's personal collection
	private int numGold;
	
	// True if the pirate is cursed, false otherwise
	private boolean isCursed;
	
	/**
	 * Constructor for a pirate, which allows the name and the
	 * amount of gold to be set
	 * (Precondition: initialGold >= 0)
	 * (Postcondition: getNumGold()  == initialGold
	 * @param name The name of the pirate
	 * @param initialGold The amount of gold in the pirate's personal collection
	 */
	public Pirate(String name, int initialGold) throws Exception {
		if(initialGold < 0)
			throw new Exception("Init Gold must be >= 0");
		this.name = name;
		this.numGold = initialGold;
		this.health = Pirate.DEFAULT_HEALTH;
		this.isCursed = false;
	}

	/**
	 * Gets the name of the pirate
	 * @return The name of the pirate
	 */
	public String getName() {
		return this.name;
	}
	
	/**
	 * Gets the cursed status of the pirate
	 * @return true if pirate is cursed, false otherwise
	 */
	public boolean getIsCursed() {
		return this.isCursed;
	}
	
	
	/**
	 * Gets the amount of health points for the pirate
	 * @return The amount of health the pirate currently has.
	 */
	public int getHealth() {
		return this.health;
	}
	
	
	/**
	 * Gets the amount of gold in the pirate's personal collection
	 * @return The amount of gold in the pirate's personal collection
	 */
	public int getNumGold() {
		return this.numGold;
	}
	
	/**
	 * Sets the isCursed flag
	 * @param isCursed The value you would like to set the isCursed flag to be 
	 */
	public void setIsCursed(boolean isCursed) {
		this.isCursed = isCursed;
	}
	
	/**
	 * Adds gold to the pirate's personal collection
	 * Precondition: numGoldToAdd >= 0
	 * Postcondition: getNumGold() >= 0
	 * @param numGoldToAdd The amount of gold to add to the pirate's personal collection
	 * @throws Exception If numGoldToAdd is negative
	 */
	public void addGold(int numGoldToAdd) throws Exception {
		this.numGold += numGoldToAdd;
	}

	/**
	 * Removes gold from the pirate's personal collection
	 * Precondition: numGoldToRemove >= 0
	 * Postcondition: getNumGold() >= 0
	 * @param numGoldToRemove The amount of gold to remove from the pirate's 
	 * personal collection
	 * @throws Exception Throws an exception if numGoldToRemove < 0 or 
	 * if numGold < numGoldToRemove
	 */
	public void removeGold(int numGoldToRemove) throws Exception {
	    if (this.numGold >= numGoldToRemove && numGoldToRemove > 0) {
	    	this.numGold -= numGoldToRemove;
	    }
	    else if (this.numGold < numGoldToRemove) {
	    	throw new Exception("Not enough gold!");
	    }
	    else {
	    	throw new Exception("You can't remove a negative amount of gold!");
	    }
	}
	
	/**
	 * Causes pirate to lose health points from an attack unless the
	 * pirate is cursed.  If a pirate is cursed, he/she is immune to
	 * damage!  
	 * The amount of health points lost is indicated in the static
	 * constant ATTACK_DAMAGE.  Once a pirate's health points are down
	 * to 0, they cannot go down any further.
	 * (Postcondition: getHealth() >= 0)
	 */
	public void loseHealthFromAttack() {
		if( !isCursed ) {
			this.health = Math.max(0,this.health-Pirate.ATTACK_DAMAGE);
		}
	}
	
	/**
	 * Attacks another pirate
	 * @param p The pirate to attack
	 */
	public void attack(Pirate p) {
		p.loseHealthFromAttack();
	}
}
