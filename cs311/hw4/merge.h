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
	int  lastIdx = findLeastStr(curStr, procNum);
	char lastStr[256];   // Keep track of the last string printed.
	sprintf(lastStr, "%s", curStr[lastIdx]);
	int  wordFreq = 0;   // Count frequency of string.

	unsigned long count = 0;   // Count number of unique strings.

	while ((lastIdx = findLeastStr(curStr, procNum)) >= 0) {
		// Print word if unique.
		if (strncmp(lastStr, curStr[lastIdx], 256) != 0) {
			fprintf(output, "%d %s", wordFreq, lastStr);
			sprintf(lastStr, "%s", curStr[lastIdx]);
			fflush(output);
			count++;
			wordFreq = 1;   // Reset word frequency.
		} else {
			wordFreq++;
		}

		// Remove recently examined word from buffer.
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

