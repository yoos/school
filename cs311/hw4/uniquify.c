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
		// usage();
	} else {
		int parentPID = getpid();
		int procNum = atoi(argv[1]);   // Number of sort processes to spawn.

		// File descriptors for pipes.
		int toSort[procNum][2];   // Parent writes to sort
		int fmSort[procNum][2];   // Parent reads from sort

		// Fork off sort processes.
		int i=0;
		while (getpid() == parentPID && i<procNum) {
			// Create pipes between parent and child sort processes.
			pipe(toSort[i]);
			pipe(fmSort[i]);

			if (fork()) {
				// Close unused ends of pipes.
				close(toSort[i][0]);
				close(fmSort[i][1]);

				printf("Forked child %d\n", i);
			} else {
				// Close unused ends of pipes.
				close(toSort[i][1]);
				close(fmSort[i][0]);

				printf("Child %d born!\n", i);
			}

			i++;
		}

		// Parse input
		printf("%lu", parse(stdin));   // TODO: This should eventually take fds of the sort processes.

		// Merge results and print
	}

	return 0;
}

