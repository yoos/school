#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <ar.h>


#define BLOCKSIZE 4096

/** Append file to archive.
 */
void append(int fd, char* name) {
	int i;
	uint8_t buffer[16];
	lseek(fd, 0*BLOCKSIZE, SEEK_SET);

	/* Check that this is an archive file. */
	if (read(fd, buffer, SARMAG) > 0) {
		for (i=0; i<SARMAG; i++) {
			if (ARMAG[i] != buffer[i]) {
				perror("This is not an archive file!");
				exit(1);
			}
		}
	}
}

/** Get a file in the archive.
 */
void get(int fd, char* name) {
}

/** Delete file from archive.
 */
void delete(int fd, char* name) {
}

/** Extract file from archive.
 */
void extract(int fd, char* name) {
}

/** Print a table of contents.
 */
void toc() {
}

/** Print usage guide.
 */
void usage() {
}


int main(int argc, char **argv)
{
	int fd, i;

	/* Has the user provided enough arguments? */
	if (argc < 3) {
		usage();
	} else {
		/* Open file. */
		fd = open(argv[2], O_WRONLY | O_CREAT | S_IRWXU);
		if (fd == -1) {
			perror("Could not open file!");
			exit(-1);
		}

		switch (*argv[1]) {

		/* Quickly append named files to archive. */
		case 'q':
			for (i=3; i<argc; i++) {
				append(fd, argv[i]);
			}
			break;

		/* Extract named files. */
		case 'x':
			for (i=3; i<argc; i++) {
				extract(fd, argv[i]);
			}
			break;

		/* Print concise table of contents of archive. */
		case 't':
			break;

		/* Print verbose table of contents of archive. */
		case 'v':
			break;

		/* Delete named files from archive. */
		case 'd':
			for (i=3; i<argc; i++) {
				delete(fd, argv[i]);
			}
			break;

		/* Quickly append all "regular" files in current directory, except the
		 * archive itself. */
		case 'A':
			break;

		/* For a given timeout, add all modified files to the archive. */
		case 'w':
			break;

		/* Print usage guide if used incorrectly. */
		default:
			usage();
		}
	}

	return 0;
}

