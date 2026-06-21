#pragma once
#include <stdint.h>
#include <stdbool.h>

bool write_physical_sector(uint8_t target_cyl, uint8_t target_head,
                           uint8_t target_sec, const uint8_t* data);

// Formats an entire track (18 sectors) on the given cylinder/head.
// Encodes the full track bitstream, syncs on INDEX, then writes it
// in one pass.  Data fields are filled with 0xE5.
// Returns true on success.
bool format_physical_track(uint8_t cyl, uint8_t head);
