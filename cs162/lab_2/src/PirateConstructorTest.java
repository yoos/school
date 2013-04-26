import org.junit.*;
import static org.junit.Assert.*;

import org.junit.Test;


public class PirateConstructorTest {

	@Test
	public void test() throws Exception {
		Pirate foo = new Pirate("foo", 127);

		assertEquals("foo", foo.getName());
		assertEquals(127, foo.getNumGold());
			
		//Pirate bar = new Pirate("bar", -255);

		foo.removeGold(27);
		
		assertEquals(100, foo.getNumGold());
	}

	@Test (expected = RuntimeException.class) public void testNegRemoveGold() throws Exception {
		Pirate foo = new Pirate("foo", 127);
		foo.removeGold(-20);
	}
}
