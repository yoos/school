/*
 *  CS 162 Exceptions Practice
 *  
 *  These are the classes you will need for the lab exercise for exceptions.
 */

import java.util.HashSet;


public class ExceptionPractice {

	// This method fills up the given StorageDevice with a bunch of junk.

	private static void fill(StorageDevice storage) {
		Thing [] things = { new Penny(), new Wallet(), new Laptop(), new FireHydrant() };
		String name = ((Thing) storage).getName();

		System.out.println("Filling the "+ name);
		for ( Thing thing : things ) {
			try {
				storage.insert(thing);
			}
			catch (ThingTooLargeException e) {
				System.out.println(e.getThing().getName() + " was too large... ignoring.");
			}
			catch (ThingTooSmallException e) {
				System.out.println(e.getThing().getName() + " was too small... throwing away.");
			}
		}
		System.out.println("The "+ name +" is full");
	}

	public static void main(String[] args) {
		Wallet w = new Wallet();
		try {
			w.bad();
		}
		catch (JustBecauseException e){
			System.out.println(e + " caught");
		}
		try {
			fill(new Wallet());
			fill(new Backpack());
			System.out.println("Success!");
		}
		catch (ThingException e) {
			System.out.println("Caught a " + e + " from a " + e.getThing().getName());
		}
	}
}


// Define exceptions here

class ThingException extends RuntimeException {
	private Thing subject;

	public ThingException(Thing t) {
		subject = t;
	}

	public Thing getThing() {
		return subject;
	}
}

class ThingTooLargeException extends ThingException {
	public ThingTooLargeException(Thing t) {
		super(t);
	}
}

class ThingTooSmallException extends ThingException {
	public ThingTooSmallException(Thing t) {
		super(t);
	}
}

class JustBecauseException extends Exception {
}




// A way of defining a "thing" to be stored.

interface Thing {
	public String getName();
}


// Things which can store other things.

abstract class StorageDevice {
	HashSet<Thing> things = new HashSet<Thing>();
	
	public void bad() throws JustBecauseException {
		throw new JustBecauseException();
	}

	public void insert(Thing t) {
		if (t instanceof FireHydrant) {
			throw new ThingTooLargeException(t);
		}
		things.add(t);
	}
}

class Backpack extends StorageDevice implements Thing {
	public String getName() {
		return "backpack";
	}
	
	public void insert(Thing t) {
		if (t instanceof Penny) {
			throw new ThingTooSmallException(t);
		}
		super.insert(t);
	}
}

class Wallet extends StorageDevice implements Thing {
	public String getName() {
		return "wallet";
	}
	
	public void insert(Thing t) {
		if (t instanceof Laptop) {
			throw new ThingTooLargeException(t);
		}
		super.insert(t);
	}
}

// Things that cannot store other things

class FireHydrant implements Thing {
	public String getName() {
		return "fire hydrant";
	}
}

class Laptop implements Thing {
	public String getName() {
		return "laptop";
	}
}

class Penny implements Thing {
	public String getName() {
		return "penny";
	}
}