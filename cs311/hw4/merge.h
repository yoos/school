#include <string.h>

// Loop through array of strings and find the "least" string with strncmp().
int findLeastStr(char* str[], int strNum)
{
	int i;
	int idx = -1;   // Index of the "least" string.

	for (i=0; i<strNum; i++) {
		// Skip null pointers.
		if (str[i] == NULL)
			continue;

		if (idx < 0) {
			idx = i;   // Set index to that of first non-null string found.
		} else {
			idx = (strncmp(str[idx], str[i], 256) < 0) ? idx : i;
		}
	}

	return idx;
}

// Merge outputs of sort processes and return number of strings printed.
unsigned long merge(FILE *output, FILE *stFmSort[][2], int procNum)
{
	int i;
	char *curStr[procNum];   // Keep track of the top elements of the sort results.

	// Allocate memory for words and initialize.
	for (i=0; i<procNum; i++) {
		curStr[i] = (char*) malloc(256*sizeof(char));
		fgets(curStr[i], 256, stFmSort[i][0]);
	}
	char lastStr[256];   // Keep track of the last string printed.
	int  lastIdx = 0;

	unsigned long count = 0;   // Count number of non-duplicate strings.

	while ((lastIdx = findLeastStr(curStr, procNum)) >= 0) {
		// Print word if not duplicate of the last.
		if (strncmp(lastStr, curStr[lastIdx], 256) != 0) {
			sprintf(lastStr, "%s", curStr[lastIdx]);
			fputs(lastStr, output);
			fflush(output);
			count++;
		}

		// Remove recently printed word from buffer.
		if (fgets(curStr[lastIdx], 256, stFmSort[lastIdx][0]) == NULL) {
			curStr[lastIdx] = 0;
		}
	}

	// Free memory.
	for (i=0; i<procNum; i++) {
		free(curStr[i]);
	}

	return count;
}

