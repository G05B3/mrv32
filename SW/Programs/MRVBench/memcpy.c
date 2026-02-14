#include "mrv32_printf.h"

#define N 64

int main() {
    int src[N];
    int dst[N];

    for (int i = 0; i < N; i++)
        src[i] = i * 3;

    for (int i = 0; i < N; i++)
        dst[i] = src[i];

    // print checksum
    int sum = 0;
    for (int i = 0; i < N; i++)
        sum += dst[i];

    mrv_printf("memcpy sum = %d\n", sum);
    return 0;
}
