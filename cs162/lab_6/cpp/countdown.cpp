#include <stdio.h>

void countdown(int);
void countdown_recursive(int);
void countup(int);

int main(void) {
    countup(10);
    return 0;
}

void countdown(int n) {
    int i;
    for (i=n; i>0; i--) {
        printf("%d\n", i);
    }
}

void countdown_recursive(int n) {
    if (n == 0) {
        return;
    }
    else {
        printf("%d\n", n);
        countdown_recursive(n-1);
    }
}

void countup(int n) {
    if (n == 0) {
        return;
    }
    else {
        countup(n-1);
        printf("%d\n", n);
    }
}

