#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>   // For memcmp.
#include <ar.h>


// TODO: This program does not check for the well-formedness of the archive
// file.

#define BLOCKSIZE 4096

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))

/* Write buffer into file.
 */
void writeToFile(int fd, char* fName, char* buffer, int bufSize)
{
	int num_written = 0;
	int location = 0;

	num_written = write(fd, buffer, bufSize);

	/* Make sure we've finished writing. */
	while (bufSize != num_written) {
		bufSize -= num_written;
		location += num_written;
		num_written = write(fd, buffer+location, bufSize);
	}

	if (num_written == -1) {
		perror("Mismatch in output.");
		unlink(fName);
		exit(-1);
	}
}

/** Append file to archive.
 */
void append(int fd, char* arName, int nNum, char** names) {
	int i;
	char buffer[16];

	/* Go to end of archive file. */
	lseek(fd, 0*BLOCKSIZE, SEEK_END);

	for (i=0; i<nNum; i++) {
		/* Open file to append. */
		int in_fd = open(names[i], O_RDONLY);
		if (in_fd == -1) {
			perror("Cannot open input file.");
			exit(-1);
		}

		/* Write ar header. */
		/* TODO: Implement // section to make this work with sizeof(arName) > 15. */
		struct stat st;
		if (fstat(in_fd, &st) == 0) {
			struct ar_hdr ah;
			sprintf(ah.ar_name, "%-16.16s", names[i]);
			sprintf(ah.ar_date, "%-12u", (uint32_t) st.st_mtime);
			sprintf(ah.ar_uid,  "%-6u",  (uint32_t) st.st_uid);
			sprintf(ah.ar_gid,  "%-6u",  (uint32_t) st.st_gid);
			sprintf(ah.ar_mode, "%-8o",  (uint32_t) st.st_mode);
			sprintf(ah.ar_size, "%-10u", (uint32_t) st.st_size);
			sprintf(ah.ar_fmag, "%s", ARFMAG);
			writeToFile(fd, arName, (char*) &ah, sizeof(ah));
		} else {
			perror("Could not stat input file.");
			exit(-1);
		}

		/* Read input file and write to archive file. */
		int num_read = 0;
		while ((num_read = read(in_fd, buffer, BLOCKSIZE)) > 0) {
			writeToFile(fd, arName, buffer, num_read);
		}

		/* Add a single newline if input filesize is odd. (This maintains
		 * compatibility with the UNIX ar.) */
		if (st.st_size % 2) {
			buffer[0] = '\n';
			writeToFile(fd, arName, buffer, 1);
		}

		close(in_fd);
	}
}

/** Find files in the archive (in order of appearance) and return number of
 *  files found.
 *  @param headers Array of headers of files found.
 */
int getHeaders(int fd, char* arName, int nNum, char** names, struct ar_hdr* headers) {
	/* Seek to first entry. */
	lseek(fd, SARMAG, SEEK_SET);

	int i;
	int num_read = 0;
	int numFound = 0;
	struct ar_hdr cur_hdr;

	/* Examine all headers until EOF. */
	while ((num_read = read(fd, (char*) &cur_hdr, sizeof(struct ar_hdr))) == sizeof(struct ar_hdr)) {
		/* Loop through the list of requested file names and find first match. */
		for (i=0; i<nNum; i++) {
			int nameLen = MIN(16, strlen(names[i]));
			if (memcmp(cur_hdr.ar_name, names[i], nameLen) == 0) {
				int dup = 0;
				int j;

				/* This is hackish, but check against list of headers for
				 * duplicates. If the header we are currently looking at is
				 * a duplicate, it will be caught here. */
				for (j=0; j<numFound; j++) {
					printf("%d %d %d \n", i, j, numFound);
					/* If the current header is a duplicate, mark it as such. */
					if (memcmp(cur_hdr.ar_name, headers[j].ar_name, nameLen) == 0)
						dup = 1;
				}

				/* If it is not a duplicate, mark as found. */
				if (dup != 1) {
					headers[numFound] = cur_hdr;   // TODO: Is this correct?
					numFound++;
				}
			}
		}

		/* Seek to next entry. Seek forward one more byte if filesize is odd.
		 * (This maintains compatibility with the UNIX ar.) */
		lseek(fd, atoi(cur_hdr.ar_size) + (atoi(cur_hdr.ar_size)%2), SEEK_CUR);
	}

	/* We're done. Seek once again to first header. */
	lseek(fd, SARMAG, SEEK_SET);

	return numFound;
}

/** Delete file from archive.
 */
void delete(int fd, char* arName, int nNum, char** names) {
}

/** Extract file from archive.
 *  TODO: Make this stop extracting after first match.
 */
void extract(int fd, char* arName, int nNum, char** names) {
	/* Get headers of the files we are looking for, if they exist. */
	struct ar_hdr headers[nNum];
	int hNum = getHeaders(fd, arName, nNum, names, headers);

	printf("%d found.\n", hNum);

	/* If no files were found, quit. */
	if (hNum == 0) {
		printf("None of the requested files were found! Exiting.\n");
		exit(0);
	}

	/* Warn the user of any files that were not found in the archive. */
	//int i, j;
	//for (i=0; i<nNum; i++) {
	//	int found = 0;
	//	for (j=0; j<hNum; j++) {
	//		int nameLen = MIN(16, strlen(names[i]));
	//		if (memcmp(headers[j].ar_name, names[i], nameLen) == 0)
	//			found = 1;
	//		if (found == 0)
	//			printf("Could not find %s\n", names[i]);
	//	}
	//}

	/* Otherwise, extract all files. */
	int num_read = 0;
	int extIndex = 0;
	struct ar_hdr cur_hdr;   // Store header for comparison.
	while ((num_read = read(fd, (char*) &cur_hdr, sizeof(struct ar_hdr))) == sizeof(struct ar_hdr)) {
		/* Should we extract this file? */
		if (memcmp(headers[extIndex].ar_name, cur_hdr.ar_name, 16) == 0) {   // TODO: Check that this works for partial matches.
			/* Increment number of files extracted. NOTE: Do this before the
			 * continue operation below! */
			extIndex++;

			/* Turn char array into string. */
			char name[16];
			sscanf(cur_hdr.ar_name, "%s", name);
			name[strlen(name)-1] = 0;

			/* If file already exists in destination directory, prompt user. */
			if (access(name, F_OK) == 0) {
				char res;
				printf("%s exists! Overwrite? (y/N) ", name);
				scanf("%c", &res);   // TODO: Sometimes, this is skipped. Why?

				if (res != 'y' && res != 'Y') {
					printf("Not extracting %s.\n", name);
					/* Seek to next entry. Seek forward one more byte if
					 * filesize is odd. (This maintains compatibility with the
					 * UNIX ar.) */
					lseek(fd, atoi(cur_hdr.ar_size) + (atoi(cur_hdr.ar_size)%2), SEEK_CUR);
					continue;
				}
			}

			/* Read octal mode. */
			int mode;
			sscanf(cur_hdr.ar_mode, "%o", &mode);

			/* Open file to write. */
			int out_fd = creat(name, mode);
			if (out_fd == -1) {
				perror("Cannot open file to write.");
				exit(-1);
			}

			/* Read file from archive and extract. */
			int out_num_read = 0;
			int out_num_to_write = atoi(cur_hdr.ar_size);
			int readSize = 0;
			char buffer[16];
			while (out_num_to_write > 0) {
				readSize = (BLOCKSIZE < out_num_to_write) ? BLOCKSIZE : out_num_to_write;
				out_num_read = read(fd, buffer, readSize);
				writeToFile(out_fd, name, buffer, out_num_read);

				out_num_to_write -= readSize;
			}

			close(out_fd);
		} else {
			/* Seek to next entry. Seek forward one more byte if filesize is
			 * odd. (This maintains compatibility with the UNIX ar.) */
			lseek(fd, atoi(cur_hdr.ar_size) + (atoi(cur_hdr.ar_size)%2), SEEK_CUR);
		}
	}
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
		int read_size = read(fd, buffer, SARMAG);
		if (read_size > 0) {
			for (i=0; i<SARMAG; i++) {
				if (ARMAG[i] != buffer[i]) {
					perror("This is not an archive file!\n");
					exit(-1);
				}
			}
		} else if (read_size == 0) {
			writeToFile(fd, argv[2], ARMAG, SARMAG);
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
				append(fd, argv[2], argc-3, argv+3);
			}
			break;

		/* Extract named files. */
		case 'x':
			if (argc < 4) {
				printf("Supply at least one file to append!\n");
			} else {
				extract(fd, argv[2], argc-3, argv+3);
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
			if (argc < 4) {
				printf("Supply at least one file to append!\n");
			} else {
				delete(fd, argv[2], argc-3, argv+3);
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

		close(fd);
	}

	return 0;
}

