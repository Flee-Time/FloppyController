#pragma once
#include <stdint.h>
#include <stdbool.h>

void lba_to_chs(uint32_t lba, uint8_t *cylinder, uint8_t *head, uint8_t *sector);
bool read_physical_sector(uint8_t target_cyl, uint8_t target_head, uint8_t target_sec, uint8_t* buffer);