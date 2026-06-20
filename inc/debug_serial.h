#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void debug_serial_init(void);
void debug_serial_write(const char* str);
void debug_serial_write_hex32(uint32_t val);
void debug_serial_write_dec32(uint32_t val);
void debug_serial_flush(void);

#ifdef __cplusplus
}
#endif
