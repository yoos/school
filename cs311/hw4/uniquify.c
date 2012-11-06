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
				printf("Forked child %d\n", i);

				// Close unused ends of pipes.
				close(fdToSort[i][0]);
				close(fdFmSort[i][1]);

				// Open read/write pipes.
				stToSort[i][1] = fdopen(fdToSort[i][1], "w");
				stFmSort[i][0] = fdopen(fdFmSort[i][0], "r");
			} else {
				printf("Child %d born!\n", i);

				//sleep(1);

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

				//while (1) {
				//	sprintf(buf, "Hello from child %d\n", i);
				//	if (fputs(buf, stFmSort[i][1]) == EOF) {
				//		perror("Child failed to write");
				//		exit(-1);
				//	}
				//	fflush(stFmSort[i][1]);

				//	if (fgets(buf, 20, stToSort[i][0]) == NULL) {
				//		perror("Child failed to read");
				//		exit(-1);
				//	}
				//	printf("Child %d read: %s\n", i, buf);
				//}
				//fclose(stToSort[i][0]);
				//printf("Child exiting.\n");

				execlp("sort", "sort", NULL);
			}

			i++;
		}

		//int j;
		//for (j=10; j>0; j--) {
		//	sprintf(buf, "Hello from parent %d\n", j);
		//	if (fputs(buf, stToSort[0][1]) == EOF) {
		//		perror("Parent failed to write");
		//		exit(-1);
		//	}
		//	fflush(stToSort[0][1]);
		//}
		//fclose(stToSort[0][1]);

		//while (1) {
		//	if (fgets(buf, 256, stFmSort[0][0]) == NULL) {
		//		perror("Parent failed to read");
		//		exit(-1);
		//	}
		//	printf("Parent read: %s\n", buf);
		//}
		//printf("Parent exiting.\n");

		// Parse input
		printf("%lu lines parsed.\n", parse(stdin, stToSort, procNum));

		// Close streams to sort
		for (i=0; i<procNum; i++) {
			fclose(stToSort[i][1]);
		}

		//for (i=0; i<procNum; i++) {
		//	printf("Read from child %d:\n", i);
		//	while (fgets(buf, 256, stFmSort[i][0]) != NULL) {
		//		printf("%s\n", buf);
		//	}
		//}

		// Merge results and print
		printf("%lu lines printed.\n", merge(stdout, stFmSort, procNum));
	}

	return 0;
}

