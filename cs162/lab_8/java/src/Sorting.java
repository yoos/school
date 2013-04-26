public class Sorting {

	private static void swap(int[] a, int i, int j) {
		int temp = a[i];
		a[i] = a[j];
		a[j] = temp;
	}

	public static void selectionSort(int[] list) {
		for (int i = 0; i < list.length - 1; i++) {
			// Find the index of the minimum value
			int minPos = i;
			for (int j = i + 1; j < list.length; j++) {
				if (list[j] < list[minPos]) {
					minPos = j;
				}
			}
			swap(list, minPos, i);
		}
	}

	public static void bubbleSort(int[] list) {
		for (int i = (list.length - 1); i >= 0; i--) {
			for (int j = 1; j <= i; j++) {
				if (list[j - 1] > list[j]) {
					// swap elements at j-1 and j
					int temp = list[j - 1];
					list[j - 1] = list[j];
					list[j] = temp;
				}
			}
		}
	}

	public static void insertionSort(int[] list) {
		for (int i = 1; i < list.length; i++) {
			int next = list[i];
			// find the insertion location while moving all larger element up
			int j = i;
			while (j > 0 && list[j - 1] > next) {
				list[j] = list[j - 1];
				j--;
			}
			// insert the element
			list[j] = next;
		}
	}

	public static void mergeSort(int [] list) {
		if (list.length <= 1) {
			return;
		}

		// Split the array in half
		int[] first = new int[list.length / 2];
		int[] second = new int[list.length - first.length];
		System.arraycopy(list, 0, first, 0, first.length);
		System.arraycopy(list, first.length, second, 0, second.length);
		
		// Sort each half
		mergeSort(first);
		mergeSort(second);

		// Merge the halves together, overwriting the original array
		merge(first, second, list);
	}

	private static void merge(int[] first, int[] second, int [] result) {
		// Merge both halves into the result array
		// Next element to consider in the first array
		int iFirst = 0;
		// Next element to consider in the second array
		int iSecond = 0;

		// Next open position in the result
		int j = 0;
		// As long as neither iFirst nor iSecond is past the end, move the
		// smaller element into the result.
		while (iFirst < first.length && iSecond < second.length) {
			if (first[iFirst] < second[iSecond]) {
				result[j] = first[iFirst];
				iFirst++;
			} else {
				result[j] = second[iSecond];
				iSecond++;
			}
			j++;
		}
		// copy what's left
		System.arraycopy(first, iFirst, result, j, first.length - iFirst);
		System.arraycopy(second, iSecond, result, j, second.length - iSecond);
	}

	// "from" and "to" specify the range of the array to sort
	private static void quicksort(int list[], int from, int to) {
		// If the indexes cross, then we've sorted the whole array.
		if (from >= to) {
			return;
		}

		// Choose a pivot value and then partition the array so that every value
		// less than the pivot is positioned before the pivot in the array and
		// every value greater than the pivot is positioned after the pivot in
		// the array.
		int pivot = list[from];
		int i = from - 1;
		int j = to + 1;
		while (i < j) {
			// Keep incrementing from the start of the range so long as the
			// values are less than the pivot.
			i++;
			while (list[i] < pivot) { i++; }
			// Keep decrementing from the end of the range so long as the values
			// are greater than the pivot.
			j--;
			while (list[j] > pivot) { j--; }
			// So long at the indexes have not crossed, swap the pivot with the
			// value that was out of place.
			if (i < j) {
				swap(list, i, j);
			}
		}

		// Recursively sort the two portions of the array
		quicksort(list, from, j);
		quicksort(list, j + 1, to);
	}

	// Helper method that kicks off the recursive quicksort method
	public static void quicksort(int [] list) {
		quicksort(list, 0, list.length-1);
	}
}
