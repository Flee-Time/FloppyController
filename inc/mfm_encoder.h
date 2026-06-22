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

// Encodes only the data+CRC portion (no sync/A1/DAM preamble).
// Used for write_physical_sector which times the write to start
// right after the existing 0xFB DAM on the track.
int mfm_encode_sector_data(const uint8_t data[512], uint32_t* output);

// Encodes a full IBM-format track (18 sectors × 512 bytes) for the
// given cylinder and head.  Includes GAP4a, GAP1, SYNC, ID AM, ID
// field with CRC, GAP2, SYNC, Data AM, 0xE5 data fill with CRC, and
// GAP3.  Returns the number of 32-bit words written to output.
// output must be at least 6500 words to hold one track.
//
// This uses a streaming packer; the static bit_buf is not touched.

int format_track_encode(uint8_t cyl, uint8_t head,
                        uint32_t* output, int max_words);
