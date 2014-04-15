#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#define SCHEDULER SCHED_RR    /*Change to SCHED_RR to use round robin */

struct sched_param param;

int main (void) {
	int i, j;
	unsigned long k = 0;
	unsigned long mask = 8; /* 4 cores */
	unsigned int len = sizeof(mask);
	pid_t pid;

	param.sched_priority = sched_get_priority_max(SCHEDULER);
	/* sets policy for current process */
	if( sched_setscheduler(0, SCHEDULER, &param ) == -1) {
		perror("sched_setscheduler failed");
		exit(-1);
	}

	if (sched_setaffinity(0, len, &mask) < 0) {
		perror("sched_setaffinity failed");
		exit(-1);
	}

	printf("START\n");

	for(i = 0; i < 4; i++) {

		switch(pid = fork()) {
            	case -1: 
		    exit(-1);

		/* case the Child executes */
		case 0:  
			j = 0;

			/* Child counts to ULONG_MAX and prints statements
	 		 * to signify halfway and completion */
    			printf("Parent: %d PID: %d\n",i,getpid());
			while(k < ULONG_MAX) {
				if (k == ULONG_MAX / 2) 
					printf("PID: %d is halfway there
					       \n",getpid());
			k++;
			}

			printf("PID: %d finished\n", getpid());
    			j++;

                	_exit(EXIT_SUCCESS);

		/* Case the parent executes (falls through) */
            	default:  
	    		break;

        	} /* End switch */
    	} /* End for loop */
	
	/* Waits on all 4 children to finish */
	for(i = 0; i < 4; i++) {
        	wait();
	}

	printf("finished\n");
	return 0;

}
