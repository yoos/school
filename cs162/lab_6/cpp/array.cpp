#include <stdio.h>
#include <stdbool.h>

#define N 10

bool contains(int*, int, int);

int main(void) {
    int data[N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    int i;

    for (i=0; i<N; i++) {
        printf("%d ", data[i]);
    }
    printf("\n");

    printf("%d\n", contains(data, N, 5));
    printf("%d\n", contains(data, N, 42));

    return 0;
}

bool contains(int list[], int size, int value) {
    for (int i=0; i<size; i++) {
        if (list[i] == value) {
            return true;
        }
    }
    return false;
}

