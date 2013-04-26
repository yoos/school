#include <stdio.h>
#include <string.h>

int main(void) {
    char str[] = {83, 116, 114, 105, 110, 103, 115, 33, 0};
    printf("%s\n", str);

    char str2[] = "C *is* simple.";
    int len = strlen(str2);
    for (int i=0; i<len; i++) {
        printf("%d\n", str2[i]);
    }

    char letter = ' ';
    while (letter < 127) {
        printf("%d %c\n", letter, letter);
        letter++;
    }
    printf("\n");


    // Task 1
    char* strA = "String A";
    char* strB = "String B";
    char* strC = "String A";

    printf("Task 1\n");
    printf("%d\n", strcmp(strA, strB));
    printf("%d\n", strcmp(strB, strC));
    printf("%d\n\n", strcmp(strA, strC));


    // Task 2
    char buffer[100];
    strcpy(buffer, strA);
    printf("Task 2\n");
    printf("%d\n\n", strlen(buffer));


    // Task 3
    char buffer2[200];
    buffer2[0] = 0;
    strcat(buffer2, strB);
    strcat(buffer2, strC);
    printf(buffer2);

    return 0;
}

