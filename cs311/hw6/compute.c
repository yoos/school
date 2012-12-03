#include	<sys/types.h>	/* basic system data types */
#include	<sys/socket.h>	/* basic socket definitions */
#include	<sys/time.h>	/* timeval{} for select() */
#include	<time.h>		/* timespec{} for pselect() */
#include	<netinet/in.h>	/* sockaddr_in{} and other Internet defns */
#include	<arpa/inet.h>	/* inet(3) functions */
#include	<errno.h>
#include	<fcntl.h>		/* for nonblocking */
#include	<netdb.h>
#include	<signal.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#include    <strings.h>     /* for bzero */
#include	<sys/stat.h>	/* for S_xxx file mode constants */
#include	<sys/uio.h>		/* for iovec{} and readv/writev */
#include	<unistd.h>
#include	<sys/wait.h>
#include	<sys/select.h>

#define MAX_LINE_LEN 4096   // Max line length
#define MAX_SOCK_ADDR 128   // Max socket address structure size
#define BUFFSIZE 8192

#define PORT 50117
#define PORT_STR "50117"


// Determine performance characteristics with timing loop.


// Ask manage.py for range.


// Receive range and compute.


// Send computed values to manage.py, then die.



int main(int argc, char **argv)
{
	int i;
	int sockfd;
	struct sockaddr_in servaddr;
	char sendline[MAX_LINE_LEN];
	char recvline[MAX_LINE_LEN];

	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	bzero(&servaddr, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);

	// Get hostname
	char hostname[256];
	if (gethostname(hostname, sizeof(hostname)) == 0)
		printf("Connecting to %s port %d.\n", hostname, ntohs(servaddr.sin_port));
	else
		perror("gethostname");
	inet_pton(AF_INET, hostname, &servaddr.sin_addr);

	connect(sockfd, (struct sockaddr*) &servaddr, sizeof(servaddr));

	// Write
	for (i=0; i<100; i++) {
		printf("Sending %dth hello.\n", i);
		sprintf(sendline, "hello %d", i);
		send(sockfd, sendline, strlen(sendline), 0);
		bzero(recvline, MAX_LINE_LEN);
		if(recv(sockfd, recvline, MAX_LINE_LEN, 0) == -1) {
			perror("server terminated prematurely");
			exit(-1);
		}
	}

	return 0;
}

