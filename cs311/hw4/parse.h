
// Parse inputs and hand out to sort processes round-robin style. Return number
// of entries parsed.
unsigned long parse(FILE *input, FILE *stToSort[][2], int procNum)
{
	char str[256];
	unsigned long count = 0;
	int proc = 0;   // Process number

	while (fgets(str, 256, input) != NULL) {
		// Feed word to sort process
		if (fputs(str, stToSort[proc][1]) == EOF) {
			perror("Failed to feed sort");
			exit(-1);
		}
		fflush(stToSort[proc][1]);

		// Calculate index of next process
		proc = (proc+1) % procNum;

		count++;
	}

	return count;
}

