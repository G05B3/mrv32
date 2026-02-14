#pragma once
#include <stdint.h>
#include <stdarg.h>
#include "mrv32_io.h"

static void mrv_print_u32(uint32_t v, unsigned base, int uppercase) {
    char buf[33];
    unsigned i = 0;

    if (v == 0) { mrv_putchar('0'); return; }

    while (v && i < sizeof(buf)) {
        uint32_t d = v % base;
        v /= base;
        if (d < 10) buf[i++] = (char)('0' + d);
        else buf[i++] = (char)((uppercase ? 'A' : 'a') + (d - 10));
    }
    while (i) mrv_putchar(buf[--i]);
}

static void mrv_print_i32(int32_t v) {
    if (v < 0) {
        mrv_putchar('-');
        uint32_t u = (uint32_t)(-(v + 1)) + 1u; /* safe for INT32_MIN */
        mrv_print_u32(u, 10, 0);
    } else {
        mrv_print_u32((uint32_t)v, 10, 0);
    }
}

static void mrv_printf(const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);

    for (const char* p = fmt; *p; p++) {
        if (*p != '%') { mrv_putchar(*p); continue; }
        p++;
        if (!*p) break;

        switch (*p) {
        case '%': mrv_putchar('%'); break;
        case 'c': { int ch = va_arg(ap, int); mrv_putchar((char)ch); } break;
        case 's': { const char* s = va_arg(ap, const char*); if (!s) s="(null)"; mrv_puts(s); } break;
        case 'd': { int v = va_arg(ap, int); mrv_print_i32((int32_t)v); } break;
        case 'u': { unsigned v = va_arg(ap, unsigned); mrv_print_u32((uint32_t)v, 10, 0); } break;
        case 'x': { unsigned v = va_arg(ap, unsigned); mrv_print_u32((uint32_t)v, 16, 0); } break;
        case 'X': { unsigned v = va_arg(ap, unsigned); mrv_print_u32((uint32_t)v, 16, 1); } break;
        default:  mrv_putchar('%'); mrv_putchar(*p); break;
        }
    }

    va_end(ap);
}