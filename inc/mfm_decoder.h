#pragma once
#include <stdint.h>
#include <stdbool.h>

void lba_to_chs(uint32_t lba, uint8_t *cylinder, uint8_t *head, uint8_t *sector);
bool read_physical_sector(uint8_t target_cyl, uint8_t target_head, uint8_t target_sec, uint8_t* buffer);
// Locate a CRC-valid, 512-byte ID field and timestamp its decoded end.
// Caller can schedule a sector write after the standard 22-byte GAP2.
bool locate_physical_sector(uint8_t cyl, uint8_t head, uint8_t sec, uint32_t* id_end_us);
uint16_t crc16_ccitt(const uint8_t* data, int len, uint16_t crc = 0xFFFF);
