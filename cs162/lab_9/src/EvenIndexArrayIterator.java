import java.util.Iterator;
import java.util.ArrayList;

public class EvenIndexArrayIterator<Type> implements Iterator<Type> {
    Object[] list;
    int index;
    final int size;

	EvenIndexArrayIterator(Type [] list) {
        size = list.length;
        this.list = new Object[size];
        for (int i=0; i<size; i++) {
            this.list[i] = list[i];
        }
        index = 0;
	}

	public boolean hasNext() {
        if (index < size) {
            return true;
        }
        else {
            return false;
        }
	}

	public Type next() {
        Type output = (Type) list[index];
        index += 2;
        return output;
	}

	public void remove() {
		throw new UnsupportedOperationException();
	}	
}
