#include <stdlib.h>
#include <stdio.h>

int main(void) {
    int n;
    printf("Please enter a number between 1 and 10: ");
    scanf("%d", &n);

    int *data = (int*) malloc(n * sizeof(int));

    for (int i=0; i<n; i++) {
        printf("Enter number for array index %d: \n", i);

        scanf("%d", &data[i]);
    }

    for (int i=0; i<n; i++) {
        printf("%d\n", data[i]);
    }

    free(data);
    data = NULL;

    return 0;
}

