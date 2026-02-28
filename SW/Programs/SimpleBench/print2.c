#include <stdint.h>
#include "mrv32_printf.h"

#define N 10

int main()
{
	int a[N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10}, b[N] = {1, 3, 5, 7, 9, 11, 13, 15, 17, 19}, c[N];
	int i;

	for (i = 0; i < N; i++)
	{
		c[i] = a[i] + b[i];
        mrv_printf("c[%d] = %d\n", i, c[i]);
	}

	return 0;

}