#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <dirent.h>
#include <ar.h>


// TODO: This program does not check for the well-formedness of the archive
// file.

#define BLOCKSIZE 4096

#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

// General-purpose character buffer.
// NOTE: Define this outside the functions or they may consistently segfault
// for inexplicable combinations of different filename lengths and filesizes
// and do other weird things that took me 20 hours to debug. Thanks to this,
// I've become good friends with gdb and valgrind.
char buffer[16];

/** Turn an array of characters into a string.
 *  TODO: This is bad, dangerous, and just wrong, but it works for now.
 */
char* stringify(int len, char* array)
{
	char out[len+1];

	sscanf(array, "%s", out);

	/* For ar_name. */
	if (out[strlen(out)-1] == '/')
	   out[strlen(out)-1] = 0;
	else
		out[strlen(out)] = 0;

	return out;
}

/** Write buffer into file.
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
 *  TODO: Check if user is trying to append the archive file to itself.
 *  Currently, append will try and succeed in making the archive file grow
 *  indefinitely in size.
 */
void append(int fd, char* arName, int nNum, char** names)
{
	int i;

	/* Go to end of archive file. */
	lseek(fd, 0*BLOCKSIZE, SEEK_END);

	//printf("names: %lu  &names: %lu  *names: %lu  &names[i]: %lu\n", names, &names, *names, &names[i]);

	for (i=0; i<nNum; i++) {
		/* Open file to append. */
		//int j;
		//printf("names[i]: ");
		//for (j=0; j<10; j++) {
		//	printf("%d ", names[i][j]);
		//}
		//printf("  &names[i]: %lu\n", &names[i]);
		//printf("Index: %d\n", i);
		//printf("Opening: %s\n\n", names[i]);

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
			writeToFile(fd, arName, "\n", 1);
		}

		close(in_fd);
	}
}

/** Find files in the archive (in order of appearance) and return number of
 *  files found.
 *  @param headers Array of headers of files found.
 */
int getHeaders(int fd, int nNum, char** names, struct ar_hdr* headers) {
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
			if (strncmp(cur_hdr.ar_name, names[i], MIN(16, strlen(names[i]))) == 0) {
				int dup = 0;
				int j;

				/* This is hackish, but check against list of headers for
				 * duplicates. If the header we are currently looking at is
				 * a duplicate, it will be caught here. */
				for (j=0; j<numFound; j++) {
					/* If the current header is a duplicate, mark it as such. */
					if (strncmp(cur_hdr.ar_name, headers[j].ar_name, 16) == 0)
						dup = 1;
				}

				/* If it is not a duplicate, mark as found. */
				if (dup != 1) {
					headers[numFound] = cur_hdr;
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
void delete(int fd, char* arName, int nNum, char** names)
{
	/* Get headers of the files we are looking for, if they exist. */
	struct ar_hdr headers[nNum];
	int hNum = getHeaders(fd, nNum, names, headers);

	/* If no files were found, quit. */
	if (hNum == 0) {
		printf("None of the requested files were found! Exiting.\n");
		exit(0);
	}

	/* Create new file. */
	char newName[strlen(arName)+10];
	sprintf(newName, "%s.tmp", arName);
	int new_fd = creat(newName, 0666);
	if (new_fd == -1) {
		perror("Could not create temporary file!");
		exit(-1);
	}

	/* Seek to first header in old archive. */
	lseek(fd, SARMAG, SEEK_SET);

	/* Write ARMAG to new archive. */
	lseek(new_fd, 0, SEEK_SET);
	writeToFile(new_fd, newName, ARMAG, SARMAG);

	/* Copy all files not in the deletion list to new file. */
	int num_read = 0;
	int delIndex = 0;
	struct ar_hdr cur_hdr;   // Store header for comparison.
	while ((num_read = read(fd, (char*) &cur_hdr, sizeof(struct ar_hdr))) == sizeof(struct ar_hdr)) {
		/* If names match, skip this file. */
		if (strncmp(headers[delIndex].ar_name, cur_hdr.ar_name, 16) == 0) {
			/* Increment number of files deleted. Effectively, we no longer
			 * check headers[delIndex] as a possible match, because we've just
			 * found the first match. */
			delIndex++;

			/* Seek to next entry. */
			lseek(fd, atoi(cur_hdr.ar_size) + (atoi(cur_hdr.ar_size)%2), SEEK_CUR);

		/* Otherwise, copy it to new archive. */
		} else {
			/* Read octal mode. */
			int mode;
			sscanf(cur_hdr.ar_mode, "%o", &mode);

			/* Read file from archive and copy to new file. */
			int out_num_read = 0;
			int out_num_to_write = atoi(cur_hdr.ar_size) + (atoi(cur_hdr.ar_size)%2);
			int readSize = 0;
			writeToFile(new_fd, newName, (char*) &cur_hdr, sizeof(struct ar_hdr));
			while (out_num_to_write > 0) {
				readSize = MIN(16, out_num_to_write);
				out_num_read = read(fd, buffer, readSize);
				writeToFile(new_fd, newName, buffer, out_num_read);
				out_num_to_write -= readSize;
			}
		}
	}

	/* Replace old archive file with new archive file. */
	unlink(arName);   /* Remove old archive file when this program dies. */
	link(newName, arName);   /* Link new to old as hard link. */
	unlink(newName);   /* Unlink new archive file. */

	close(new_fd);
}

/** Extract file from archive.
 */
void extract(int fd, int nNum, char** names)
{
	/* Get headers of the files we are looking for, if they exist. */
	struct ar_hdr headers[nNum];
	int hNum = getHeaders(fd, nNum, names, headers);

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
	//		if (strncmp(headers[j].ar_name, names[i], MIN(16, strlen(names[i]))) == 0)
	//			found = 1;
	//		if (found == 0)
	//			printf("Could not find %s\n", names[i]);
	//	}
	//}

	/* Otherwise, extract all files. */
	int num_read = 0;
	int extIndex = 0;
	struct ar_hdr cur_hdr;   // Store header for comparison.
	while ((num_read = read(fd, (char*) &cur_hdr, sizeof(struct ar_hdr))) == sizeof(struct ar_hdr) && extIndex < hNum) {
		/* Should we extract this file? */
		if (strncmp(headers[extIndex].ar_name, cur_hdr.ar_name, 16) == 0) {
			/* Increment number of files extracted. NOTE: Do this before the
			 * continue operation below! */
			extIndex++;

			/* Turn char array into string. */
			char name[16];
			sscanf(cur_hdr.ar_name, "%s", name);
			if (name[strlen(name)-1] == '/')
				name[strlen(name)-1] = 0;
			else
				name[strlen(name)] = 0;

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
			while (out_num_to_write > 0) {
				readSize = MIN(BLOCKSIZE, out_num_to_write);
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
void toc(int fd, int verbose)
{
	int num_read = 0;
	struct ar_hdr cur_hdr;

	/* Seek to first header. */
	lseek(fd, SARMAG, SEEK_SET);

	while ((num_read = read(fd, (char*) &cur_hdr, sizeof(struct ar_hdr))) == sizeof(struct ar_hdr)) {
		if (verbose == 1) {
			/* Generate mode string. */
			char mode[10];
			int modeInt;
			sscanf(cur_hdr.ar_mode, "%o", &modeInt);
			mode[9] = 0;
			int i;
			for (i=9; i>0; i--) {
				if ((modeInt & 1) == 1) {
					if (i%3 == 0)
						mode[i-1] = 'x';
					else if (i%3 == 1)
						mode[i-1] = 'r';
					else
						mode[i-1] = 'w';
				} else {
					mode[i-1] = '-';
				}
				modeInt = modeInt>>1;
			}

			/* Generate time string. */
			char date[30];
			time_t fileTime = atoi(cur_hdr.ar_date);
			struct tm* timeinfo;
			timeinfo = localtime(&fileTime);
			strftime(date, 30, "%b %d %R %Y", timeinfo);

			printf("%s %s/%s %10.10s %s ", mode, stringify(6, cur_hdr.ar_uid), stringify(6, cur_hdr.ar_gid), stringify(10, cur_hdr.ar_size), date);
		}

		/* Stringify name. */
		char name[16];
		sscanf(cur_hdr.ar_name, "%s", name);
		if (name[strlen(name)-1] == '/')
		   name[strlen(name)-1] = 0;
		else
			name[strlen(name)] = 0;
		printf("%s\n", name);

		/* Seek to next header. */
		lseek(fd, atoi(cur_hdr.ar_size) + (atoi(cur_hdr.ar_size)%2), SEEK_CUR);
	}
}

/** Print usage guide.
 */
void usage()
{
	printf("Usage:\n");
	exit(-1);
}


int main(int argc, char **argv)
{
	int fd, i;
	DIR* dp;
	struct dirent* ep;

	/* Has the user provided enough arguments? */
	if (argc < 3) {
		printf("Too few arguments!\n");
		usage();
	} else if (strlen(argv[1]) > 1) {
		printf("Too many keys!\n");
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
				//printf("&argv: %lu, argv[3]: %lu, &argv[3]: %lu, *argv[3]: %lu\nargv[4]: %lu, argv[5]: %lu\n", &argv, argv[3], &argv[3], *argv[3], argv[4], argv[5]);
				append(fd, argv[2], argc-3, argv+3);
			}
			break;

		/* Extract named files. */
		case 'x':
			if (argc < 4) {
				printf("Supply at least one file to extract!\n");
			} else {
				extract(fd, argc-3, argv+3);
			}
			break;

		/* Print concise table of contents of archive. */
		case 't':
			toc(fd, 0);
			break;

		/* Print verbose table of contents of archive. */
		case 'v':
			toc(fd, 1);
			break;

		/* Delete named files from archive. */
		case 'd':
			if (argc < 4) {
				printf("Supply at least one file to delete!\n");
			} else {
				delete(fd, argv[2], argc-3, argv+3);
			}
			break;

		/* Quickly append all regular files in current directory, except the
		 * archive itself. */
		case 'A':
			dp = opendir ("./");

			if (dp != NULL) {
				while ((ep = readdir(dp))) {
					/* Skip certain file ep->d_name. */
					if (strcmp(ep->d_name, ".") == 0 ||
							strcmp(ep->d_name, "..") == 0 ||
							strcmp(ep->d_name, argv[2]) == 0)
						continue;

					/* Skip non-regular files. */
					struct stat st;
					if (stat(ep->d_name, &st) == 0) {
						if (!S_ISREG(st.st_mode))
							continue;
					}

					/* Append files. */
					char* fname = &ep->d_name;
					//printf("%s d_name has address %lu and file at %lu points to %lu\n", ep->d_name, &ep->d_name, &fname, fname);
					append(fd, argv[2], 1, &fname);
				}

				closedir(dp);
			} else {
				perror("Could not open directory.");
				exit(-1);
			}
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

