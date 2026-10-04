#pragma once
#include <stdint.h>
using absolute_time_t = uint64_t;
absolute_time_t get_absolute_time();
uint32_t to_ms_since_boot(absolute_time_t);
uint32_t time_us_32();
void sleep_us(uint64_t);
void busy_wait_us_32(uint32_t);
void tight_loop_contents();
bool gpio_get(unsigned);
void __dmb();
