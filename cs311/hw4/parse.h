
unsigned long parse(FILE *input, FILE *stToSort[][2], int procNum)
{
	char str[256];
	unsigned long count = 0;
	int proc = 0;   // Process number

	// Hand out entries to sort processes, round-robin style.
	while (fgets(str, 256, input) != NULL) {
		if (fputs(str, stToSort[proc][1]) == EOF) {
			perror("Failed to feed sort");
			exit(-1);
		}
		fflush(stToSort[proc][1]);

		proc = (proc+1) % procNum;
		count++;
	}

	return count;
}

