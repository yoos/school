import static org.junit.Assert.*;

import org.junit.Test;


public class TreasureChestTest {

	@Test
	public void test() throws Exception {
		/** TreasureChest() test
		 *  
		 *  For a TreasureChest object tc, we need to check that it is initialized correctly
		 *  by checking that tc.getNumGold == initialGold. We must also check that the method
		 *  throws an exception if initialGold < 0.
		 */
		Pirate foo = new Pirate("foo", 1000);
		TreasureChest tc = new TreasureChest(1000);
		assertEquals(1000, tc.checkGold());
		
		//TreasureChest badTC = new TreasureChest(-1000);
		
		/** addGold() test
		 *  
		 *  The numGoldToAdd must be positive, which means tc.getNumGold() after the
		 *  operation must be greater than that before the operation. Similarly, the pirate
		 *  must have tc.getNumGold() less gold after the operation.
		 *  
		 *  If numGoldToAdd is negative, or if the pirate tries to add more gold than he has,
		 *  the method should throw an exception.
		 */
		
		/** removeGold() test
		 *  
		 *  The numGoldToAdd must be positive, which means tc.getNumGold() after the
		 *  operation must be less than that before the operation. Similarly, the pirate
		 *  must have tc.getNumGold() more gold after the operation. In addition, the pirate
		 *  should be cursed after the method call.
		 *  
		 *  If numGoldToAdd is negative, or if the chest has less gold than the pirate is
		 *  trying to remove, the method should throw an exception.
		 */

		/** checkGold() test
		 * 
		 *  tc.checkGold() should return numGold.
		 */
		tc.addGold(100, foo);
		assertEquals(1100, tc.checkGold());
	}

}
