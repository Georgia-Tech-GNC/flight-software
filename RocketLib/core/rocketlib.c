#include "rocketlib.h"

void run_rocket_os() {
    while (1) { }
}

void __attribute__((weak)) debug(const char *restrict format, ...) {
    UNUSED(format);
}