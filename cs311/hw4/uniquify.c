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
		int i;
		for (i=0; i<atoi(argv[1]); i++) {
			// Fork off multiple sort processes
			printf("Fork %d\n", i);
		}

		// Parse input
		printf("%lu", parse(stdin));   // TODO: This should eventually take fds of the sort processes.

		// Merge results and print
	}

	return 0;
}
