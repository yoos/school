#include <stdio.h>

int main(void) {
    int data[] = {1, 2, 3, 4, 5, 6, 7, 8};
    printf("The data is stored at: %p\n", data);

    int *data_ptr = data;
    printf("The data_ptr points to: %p\n", data_ptr);


    printf("\nLooping through array 1 byte at a time:\n\n");
    int i;
    for (i=0; i<8; i++) {
        printf("%d = %d\n", data[i], *data_ptr);
        printf("Moving forward 1 integer which is %d bytes of memory\n", sizeof(int));
        printf("%p -> %p\n", data_ptr, data_ptr+1);
        data_ptr = data_ptr+1;
    }

    data_ptr = &data;
    printf("\nLooping through array 2 bytes at a time:\n\n");
    for (i=0; i<4; i++) {
        printf("%d = %d\n", data[i*2], *data_ptr);
        printf("Moving forward 2 integers which is %d bytes of memory\n", sizeof(int)*2);
        printf("%p -> %p\n", data_ptr, data_ptr+2);
        data_ptr = data_ptr+2;
    }

    data_ptr = &data[3];
    printf("\nLooping through array 1 byte at a time from data[3]:\n\n");
    for (i=0; i<4; i++) {
        printf("%d = %d\n", data[i+3], *data_ptr);
        printf("Moving forward 1 integer which is %d bytes of memory\n", sizeof(int));
        printf("%p -> %p\n", data_ptr, data_ptr+1);
        data_ptr = data_ptr+1;
    }



    return 0;
}
