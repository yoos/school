/**
 * This class represents a Treasure Chest, which has the ultimate
 * purpose of storing gold.
 */

public class TreasureChest {

	// The amount of gold in the treasure chest
	private int numGold;
	
	/**
	 * Constructor for the treasure chest
	 * Precondition: initialGold must be >= 0
	 * Postcondition: getNumGold()  == initialGold
	 * @param initialGold The initial amount of gold in the chest
	 * @throws Exception if the initialGold is < 0
	 */
	public TreasureChest(int initialGold) throws Exception {
		if( initialGold < 0 ) {
			throw new Exception("Initial gold must be >= 0");
		}
		this.numGold = initialGold;
	}
	
	/**
	 * Adds gold to chest and subtracts the amount added to the treasure chest from pirate's
	 * personal gold collection.
	 * Precondition: numGoldToAdd >= 0, p.getNumGold() >= numGoldToAdd
	 * Postcondition: getNumGold() >= 0, p.getNumGold() >= 0
	 * @param numGoldToAdd The amount of gold to add
	 * @param p The pirate that contributes gold to the chest.  This 
	 * pirate's personal gold collection will be reduced by the amount in numGoldToAdd
	 * @throws Exception If the amount of gold added to the chest is < 0 or if
	 * the pirate's personal collection is less than numGoldToAdd
	 */
	public void addGold(int numGoldToAdd, Pirate p) throws Exception {
		if( numGoldToAdd < 0 ) {
			throw new Exception("Cannot add negative amounts of gold");
		}
		p.removeGold(numGoldToAdd);
		this.numGold += numGoldToAdd;
	}

	/**
	 * Removes gold from the chest.
	 * Precondition: numGoldToRemove >= 0
	 * Postcondition: getNumGold() >= 0 and Pirate's personal collection of gold must be >= 0
	 * @param numGoldToRemove The amount of gold to remove from the chest.
	 * @param p The pirate that removes gold from the chest.  This pirate's
	 * personal collection of gold will increase by the amount in numGoldToRemove.
	 * However, the pirate also becomes cursed.
	 * @throws Exception Throws an exception if the amount numGoldToRemove is < 0 and
	 * if numGoldToRemove exceeds the amount of gold in the chest
	 */
	public void removeGold(int numGoldToRemove, Pirate p) throws Exception {
		if( numGoldToRemove < 0 ) {
			throw new Exception("Cannot remove negative amounts of gold");
		}
		if( numGold < numGoldToRemove ) {
			throw new Exception("Not enough gold in chest");
		}
		this.numGold -= numGoldToRemove;
		p.setIsCursed(true);
		p.addGold(numGoldToRemove);
	}

	/**
	 * Checks the amount of gold in the treasure chest
	 * @return The amount of gold in the treasure chest
	 */
	public int checkGold() {
		return this.numGold;
	}
	
}

