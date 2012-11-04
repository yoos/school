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

		FILE sort_fd[procNum];

		// Fork off sort processes.
		int i=0;
		while (getpid() == parentPID && i<procNum) {
			if (fork()) {
				// Do parent stuff
				printf("Forked child %d\n", i);
			} else {
				// Do child stuff
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

