
public class PirateConstructorHarness {
	public static void main(String[] args)
	throws Exception {
		try {
			Pirate foo = new Pirate("bar", 127);
			foo.getName();
			foo.getNumGold();
		}
		catch (Exception e) {
			System.out.println();
		}
	}
}
