#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>


/** Append file to archive.
 */
void append() {
}

/** Get a file in the archive.
 */
void get() {
}

/** Delete file from archive.
 */
void delete() {
}

/** Extract file from archive.
 */
void extract() {
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
	/* Has the user provided enough arguments? */
	if (argc < 3) {
		usage();
	} else {
		switch (argv[1]) {

		/* Quickly append named files to archive. */
		case 'q':
			break;

		/* Extract named files. */
		case 'x':
			break;

		/* Print concise table of contents of archive. */
		case 't':
			break;

		/* Print verbose table of contents of archive. */
		case 'v':
			break;

		/* Delete named files from archive. */
		case 'd':
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

	return 0;
}

