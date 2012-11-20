#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include <num_primes.h>

int main(int argc, char** argv)
{
	num_primes(   1, 1000);
	num_primes(1001, 2000);
	num_primes(2001, 3000);
	num_primes(3001, 4000);

	return 0;
}

