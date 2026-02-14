#pragma once
#include <stdint.h>
#include "mrv_memmap.h"

static inline void mrv_putchar(char c) {
    *(volatile uint8_t*)MRV_UART_TX = (uint8_t)c;
}

static inline void mrv_puts(const char* s) {
    while (*s) mrv_putchar(*s++);
}

static inline void mrv_exit(uint32_t code) {
    *(volatile uint32_t*)MRV_TOHOST = code;
    for(;;) { } /* should not return */
}