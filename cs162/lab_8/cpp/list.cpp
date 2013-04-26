#include <stdlib.h>
#include <stdio.h>

int main(void) {
    int n = 0;

    int* data[2];

    data[0] = (int*) malloc(sizeof(int));

    int io_index = 0;

    while (1) {
        printf("Enter number for array index %d: \n", n);
        scanf("%d", &data[io_index][n]);

        if (data[io_index][n] == -1) {
            break;
        }
        else {
            data[1-io_index] = (int*) malloc((n+2) * sizeof(int));
            for (int i=0; i<n+1; i++) {
                data[1-io_index][i] = data[io_index][i];
                //printf("data[%d][%d]: %d   data[%d][%d]: %d\n", 1-io_index, i, data[1-io_index][i], io_index, i, data[io_index][i]);
            }
            free(data[io_index]);
            io_index = 1-io_index;
            n++;
        }
    }

    for (int i=0; i<n; i++) {
        printf("%d\n", data[io_index][i]);
    }

    //free(data[io_index]);
    //free(data[1-io_index]);
    //data[io_index] = NULL;
    //data[1-io_index] = NULL;

    return 0;
}

