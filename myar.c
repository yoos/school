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
void append(int fd, char* arName, char* inName) {
	int i;
	uint8_t buffer[16];

	/* Open file to append. */
	int in_fd = open(inName, O_RDONLY);
	if (in_fd == -1) {
		perror("Cannot open input file.");
		exit(-1);
	}

	/* Go to end of file. */
	lseek(fd, 0*BLOCKSIZE, SEEK_END);

	/* Read input file and write to archive file. */
	int num_read = 0;
	int num_written = 0;
	int location = 0;
	while ((num_read = read(in_fd, buffer, BLOCKSIZE)) > 0) {
		num_written = write(fd, buffer, num_read);

		/* Make sure we've finished writing. */
		while (num_read != num_written) {
			num_read -= num_written;
			location += num_written;
			num_written = write(fd, buffer+location, num_read);
		}

		if (num_written == -1) {
			perror("Mismatch in output.");
			unlink(arName);
			exit(-1);
		}

		lseek(in_fd, BLOCKSIZE, SEEK_CUR);
	}

	close(in_fd);
}

/** Get a file in the archive.
 */
void get(int fd, char* arName, char* name) {
}

/** Delete file from archive.
 */
void delete(int fd, char* arName, char* name) {
}

/** Extract file from archive.
 */
void extract(int fd, char* arName, char* name) {
}

/** Print a table of contents.
 */
void toc() {
}

/** Print usage guide.
 */
void usage() {
	printf("Usage:\n");
	exit(-1);
}


int main(int argc, char **argv)
{
	int fd, i;
	char buffer[16];

	/* Has the user provided enough arguments? */
	if (argc < 3) {
		printf("Too few arguments!\n\n");
		usage();
	} else {
		/* Set file mode creation mask to 0. */
		umask(0);

		/* Open file. */
		fd = open(argv[2], O_RDWR | O_CREAT, 0666);
		if (fd == -1) {
			perror("Could not open archive file!");
			exit(-1);
		}

		/* Check that this is an archive file. */
		lseek(fd, 0*BLOCKSIZE, SEEK_SET);
		if (read(fd, buffer, SARMAG) >= 0) {
			for (i=0; i<SARMAG; i++) {
				if (ARMAG[i] != buffer[i]) {
					perror("This is not an archive file!\n");
					exit(-1);
				}
			}
		} else {
			perror("Could not read archive file!");
			exit(-1);
		}

		switch (*argv[1]) {

		/* Quickly append named files to archive. */
		case 'q':
			if (argc < 4) {
				printf("Supply at least one file to append!\n");
			} else {
				printf("Appending file to archive.\n");
				for (i=3; i<argc; i++) {
					append(fd, argv[2], argv[i]);
				}
			}
			break;

		/* Extract named files. */
		case 'x':
			for (i=3; i<argc; i++) {
				extract(fd, argv[2], argv[i]);
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
				delete(fd, argv[2], argv[i]);
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

