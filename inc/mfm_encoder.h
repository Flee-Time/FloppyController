#pragma once
#include <stdint.h>
#include <stdbool.h>

// Encodes 512 bytes of data into a raw MFM half-cell bitstream ready
// for the flux_writer PIO. The bitstream includes sync field, A1×3
// address marks with missing clocks, the FB data address mark, the
// 512 data bytes, and a 2-byte CRC-16/CCITT.
//
// Returns the number of 32-bit words written to output.
// output must be at least 300 words (9600 bits, ~530 MFM bytes × 16).

int mfm_encode_sector(const uint8_t data[512], uint32_t* output);
