// Return number of primes up to given number n.
// NOTE: This was originally intended to only return the number of primes, not
// print them out. I missed that last detail until after I had finished it.
uint64_t num_primes(uint64_t n)
{
	uint64_t i;

	if (n < 2) {
		printf("There are no primes up to %lu.\n", n);
		return 0;
	}

	// Set up array for "marking" numbers as prime (1) or composite (0).
	int* primeMarks = (int*) malloc(sizeof(int) * n+1);
	for (i=2; i<=n; i++) {
		primeMarks[i] = 1;
	}

	// Sift out the primes.
	uint64_t numPrimes = 0;   // Number of primes, starting with 2.
	uint64_t k = 1;
	uint64_t sqrtN = sqrt(n);
	while (k <= sqrtN) {
		// Find next prime.
		do {
			k++;
		} while (primeMarks[k] == 0 && k <= sqrtN);

		if (k <= sqrtN) {
			// Mark all other positive multiples of current number, up to n, as
			// composites.
			for (i=2; i<=(n/k); i++) {
				primeMarks[i*k] = 0;
			}
		}
	}

	// Count the number of primes.
	for (i=2; i<=n; i++) {
		if (primeMarks[i] == 1) {
			numPrimes++;
		}
	}

	// Print result.
	printf("There are %lu primes up to %lu. They are:\n", numPrimes, n);
	//for (i=2; i<=n; i++) {
	//	if (primeMarks[i]) {
	//		printf("%d ", (int) i);
	//	}
	//}
	printf("\n");

	free(primeMarks);

	return numPrimes;
}


