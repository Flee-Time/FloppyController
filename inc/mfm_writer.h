#pragma once
#include <stdint.h>
#include <stdbool.h>

bool write_physical_sector(uint8_t target_cyl, uint8_t target_head,
                           uint8_t target_sec, const uint8_t* data);
