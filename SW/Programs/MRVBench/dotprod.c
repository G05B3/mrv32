#include "mrv32_printf.h"

#define N 16

int main() {
    int a[N], b[N];
    for (int i = 0; i < N; i++) {
        a[i] = i;
        b[i] = N - i;
    }

    int acc = 0;
    for (int i = 0; i < N; i++)
        acc += a[i] * b[i]; // later MUL, now emulate via loop if needed

    mrv_printf("dot = %d\n", acc);
    return 0;
}