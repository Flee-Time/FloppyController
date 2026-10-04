#pragma once
#include <stdint.h>
#include <stdbool.h>

constexpr int MFM_SECTOR_WORDS = 266;      // 532 MFM bytes, including GAP3 splice
constexpr int MFM_SECTOR_DATA_WORDS = 257; // 514 MFM bytes
constexpr int MFM_TRACK_MIN_WORDS = 6003;  // 11990 bytes of records + >=16 tail bytes
constexpr int MFM_TRACK_MAX_WORDS = 6500;

// Encodes 512 bytes of data into a raw MFM half-cell bitstream ready
// for the flux_writer PIO. The bitstream includes sync field, A1×3
// address marks with missing clocks, the FB data address mark, the
// 512 data bytes, a 2-byte CRC-16/CCITT, and two GAP3 splice bytes.
// Words are MSB first, matching the PIO's left-shifting output register.
//
// Returns the number of 32-bit words written to output.
// output must be at least MFM_SECTOR_WORDS words.

int mfm_encode_sector(const uint8_t data[512], uint32_t* output);

// Encodes only the data+CRC portion (no sync/A1/DAM preamble).
// output must be at least MFM_SECTOR_DATA_WORDS words.
int mfm_encode_sector_data(const uint8_t data[512], uint32_t* output);

// Encodes a full IBM-format track (18 sectors × 512 bytes) for the
// given cylinder and head.  Includes GAP4a, GAP1, SYNC, ID AM, ID
// field with CRC, GAP2, SYNC, Data AM, 0xE5 data fill with CRC, and
// GAP3. Encodes exactly max_words words, filling the remainder with GAP4b.
// Returns -1 if max_words is too small to hold every sector and a tail gap.
//
// Sector and format encoders are intended to run on the floppy worker core.

int format_track_encode(uint8_t cyl, uint8_t head,
                        uint32_t* output, int max_words);
