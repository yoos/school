
unsigned long parse(FILE *input)
{
	char str[100];
	unsigned long count = 0;

	while (fgets(str, 100, input) != NULL) {
		fputs(str, stdout);
		count++;
	}

	return count;
}

