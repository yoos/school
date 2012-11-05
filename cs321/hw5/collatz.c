#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


int collatz(int x)
{
	if (x == 1)     return 1;
	if (x % 2 == 0) return x/2;
	else            return 3*x+1;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		printf("Too few arguments.\n");
		exit(-1);
	}

	int x, i;
	int num;

	for (num=1; num<=atoi(argv[1]); num++) {
		x = num;
		i = 0;

		while (x != 1) {
			x = collatz(x);
			i++;
		}

		printf("%d, %d\n", num, i);
	}

	return 0;
}

