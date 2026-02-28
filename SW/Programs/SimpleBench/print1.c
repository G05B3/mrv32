#include <stdint.h>
#include "mrv32_printf.h"

int main() {
    mrv_printf("Hello MRV32! a=%d c=%c hex=%x str=%s %%\n", -42, 'Z', 0xBEEF, "ok");
    return 0; // crt0 writes this to TOHOST
}