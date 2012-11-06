#include <string.h>

//int notDone(char* str[], int strNum)
//{
//	int i;
//	for (i=0; i<strNum; i++) {
//		if (str[i] > 0) {
//			return 1;
//		}
//	}
//	return 0;
//}

int findLeastStr(char* str[], int strNum)
{
	int i;
	int idx = -1;   // Index of the "least" string.

	for (i=0; i<strNum; i++) {
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

unsigned long merge(FILE *output, FILE *stFmSort[][2], int procNum)
{
	int i;
	char *curStr[procNum];   // Keep track of the top elements of the sort results.
	for (i=0; i<procNum; i++) {
		curStr[i] = (char*) malloc(256*sizeof(char));
		fgets(curStr[i], 256, stFmSort[i][0]);
	}
	char lastStr[256];   // Last string printed.
	int  lastIdx = 0;

	unsigned long count = 0;

	while ((lastIdx = findLeastStr(curStr, procNum)) >= 0) {
		sprintf(lastStr, "%s", curStr[lastIdx]);
		fputs(lastStr, output);
		fflush(output);
		if (fgets(curStr[lastIdx], 256, stFmSort[lastIdx][0]) == NULL) {
			curStr[lastIdx] = 0;
		}
		count++;
	}

	for (i=0; i<procNum; i++) {
		free(curStr[i]);
	}

	return count;
}

