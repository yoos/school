#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>

#include "parse.h"
#include "merge.h"
#include "usage.h"

#define PAR_NUM 3   // Number of parallel sort processes

int main(int argc, char **argv)
{
	if (argc != 2) {
		usage();
	} else {
		int parentPID = getpid();
		int procNum = atoi(argv[1]);   // Number of sort processes to spawn.
		char buf[256];

		// File descriptor numbers for pipes.
		int fdToSort[procNum][2];   // Parent writes to sort
		int fdFmSort[procNum][2];   // Parent reads from sort

		// Streams
		FILE *stToSort[procNum][2];
		FILE *stFmSort[procNum][2];

		// Fork off sort processes.
		int i=0;
		while (getpid() == parentPID && i<procNum) {
			// Create pipes between parent and child sort processes.
			pipe(fdToSort[i]);
			pipe(fdFmSort[i]);

			if (fork()) {
				// Close unused ends of pipes.
				close(fdToSort[i][0]);
				close(fdFmSort[i][1]);

				// Open read/write pipes.
				stToSort[i][1] = fdopen(fdToSort[i][1], "w");
				stFmSort[i][0] = fdopen(fdFmSort[i][0], "r");
			} else {
				// Close unused ends of pipes.
				close(fdToSort[i][1]);
				close(fdFmSort[i][0]);

				// Redirect stdin/stdout.
				if (dup2(fdToSort[i][0], STDIN_FILENO) == -1) {
					perror("Failed to redirect stdin");
				}
				if (dup2(fdFmSort[i][1], STDOUT_FILENO) == -1) {
					perror("Failed to redirect stdout");
				}

				// Open read/write pipes.
				stToSort[i][0] = fdopen(fdToSort[i][0], "r");
				stFmSort[i][1] = fdopen(fdFmSort[i][1], "w");

				execlp("sort", "sort", NULL);
			}

			i++;
		}

		// Parse input
		long unsigned int parseCount = parse(stdin, stToSort, procNum);

		// Close streams to sort
		for (i=0; i<procNum; i++) {
			fclose(stToSort[i][1]);
		}

		// Merge results and print
		long unsigned int mergeCount = merge(stdout, stFmSort, procNum);

		// Print results.
		printf("\n%lu lines parsed.\n", parseCount);
		printf("%lu lines printed.\n", mergeCount);
	}

	return 0;
}

