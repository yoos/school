int main(void) {
    int x = 42;
    printf("x: %d\n", x);
    printf("x address: %p\n", &x);

    x = 11;
    printf("x: %d\n", x);
    printf("x address: %p\n", &x);

    int *int_ptr = &x;
    printf("int_ptr points to: %d\n", x);
    printf("memory address %p contains: %d\n", int_ptr, *int_ptr);

    *int_ptr = 101;
    printf("x has value: %d\n", x);
    printf("memory address %p contains value %d\n", int_ptr, *int_ptr);

    return 0;
}
