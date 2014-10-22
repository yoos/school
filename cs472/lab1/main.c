#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>

int main(int argc, char **argv)
{
	printf("CS472 Lab 1 benchmark\n");

	float a=0;
	int b=0;

	long long int i;
	for (i=0; i<10000000000; i++) {
		//a /= 1.0f;
		b += 1;
	}

	return 0;
}

