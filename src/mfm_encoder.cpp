#include "mfm_encoder.h"
#include <string.h>

// CRC-16/CCITT (poly 0x1021, init 0xFFFF)
static uint16_t crc16(const uint8_t* data, int len, uint16_t crc) {
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

// Buffer for accumulating half-cell bits before packing into 32-bit words.
#define MAX_BITS 10000
static uint8_t bit_buf[MAX_BITS];
static int bit_count = 0;

// Emit a single half-cell bit (0 = no transition, 1 = flux reversal).
static void emit(uint8_t b) {
    if (bit_count < MAX_BITS)
        bit_buf[bit_count++] = b;
}

// Emit "count" zero bits (no transitions).
static void emit_zeros(int count) {
    while (count-- > 0) emit(0);
}

// Emit the raw MFM bit pattern for a single byte with normal clock encoding.
// prev_data: 1 if the previous byte's last data bit was a 1, else 0.
// Returns: the value of this byte's last data bit (for chaining).
static int emit_byte_normal(uint8_t byte, int prev_data) {
    for (int i = 7; i >= 0; i--) {
        int data_bit = (byte >> i) & 1;
        if (data_bit) {
            emit(0); emit(1);  // no clock, data transition
        } else {
            if (prev_data == 0) {
                emit(1); emit(0);  // clock at start, no data
            } else {
                emit(0); emit(0);  // no clock, no data
            }
        }
        prev_data = data_bit;
    }
    return prev_data;
}

// Emit the raw MFM bit pattern for an A1 sync byte with missing clock.
// The missing clock is on the second bit cell (first 0 after a 1).
static void emit_byte_a1(void) {
    // A1 = 0xA1 = 0b10100001. Missing clock on bit 6 (second data bit).
    emit(0); emit(1);  // bit7=1: no clock, data
    emit(0); emit(0);  // bit6=0: MISSING clock, no data
    emit(0); emit(1);  // bit5=1: no clock, data
    emit(0); emit(0);  // bit4=0: no clock (prev=1), no data
    emit(1); emit(0);  // bit3=0: clock (prev=0), no data
    emit(1); emit(0);  // bit2=0: clock (prev=0), no data
    emit(1); emit(0);  // bit1=0: clock (prev=0), no data
    emit(0); emit(1);  // bit0=1: no clock, data
}

int mfm_encode_sector(const uint8_t data[512], uint32_t* output) {
    bit_count = 0;

    // 12 bytes of 0x00 sync (96 cells). Previous data bit = 0 (assume).
    int prev = 0;
    for (int i = 0; i < 12; i++)
        prev = emit_byte_normal(0x00, prev);

    // A1 A1 A1 with missing clocks
    emit_byte_a1();
    emit_byte_a1();
    emit_byte_a1();

    // FB data address mark (normal encoding, prev = last bit of A1 = 1)
    prev = emit_byte_normal(0xFB, 1);

    // 512 data bytes
    for (int i = 0; i < 512; i++)
        prev = emit_byte_normal(data[i], prev);

    // CRC-16 over A1×3 + FB + 512 bytes
    uint8_t crc_buf[4] = {0xA1, 0xA1, 0xA1, 0xFB};
    uint16_t crc = crc16(crc_buf, 4, 0xFFFF);
    crc = crc16(data, 512, crc);

    // CRC bytes (MSB first on disk)
    emit_byte_normal((crc >> 8) & 0xFF, prev);
    emit_byte_normal(crc & 0xFF, (crc >> 8) & 1);

    // Pad to fill any partial 32-bit word at the end.
    while (bit_count & 31) emit(0);

    // Pack bits into 32-bit words, MSB first within each word.
    int word_count = 0;
    int bi = 0;
    while (bi < bit_count) {
        uint32_t w = 0;
        for (int i = 0; i < 32 && bi < bit_count; i++, bi++) {
            if (bit_buf[bi]) w |= (1u << i);
        }
        output[word_count++] = w;
    }

    return word_count;
}

int mfm_encode_sector_data(const uint8_t data[512], uint32_t* output) {
    bit_count = 0;

    // CRC-16 over A1×3 + FB + 512 bytes
    uint8_t crc_buf[4] = {0xA1, 0xA1, 0xA1, 0xFB};
    uint16_t crc = crc16(crc_buf, 4, 0xFFFF);
    crc = crc16(data, 512, crc);

    // 512 data bytes (prev = last bit of 0xFB = 1)
    int prev = 1;
    for (int i = 0; i < 512; i++)
        prev = emit_byte_normal(data[i], prev);

    // CRC bytes (MSB first on disk)
    emit_byte_normal((crc >> 8) & 0xFF, prev);
    emit_byte_normal(crc & 0xFF, (crc >> 8) & 1);

    // Pad to fill any partial 32-bit word at the end.
    while (bit_count & 31) emit(0);

    // Pack bits into 32-bit words, MSB first within each word.
    int word_count = 0;
    int bi = 0;
    while (bi < bit_count) {
        uint32_t w = 0;
        for (int i = 0; i < 32 && bi < bit_count; i++, bi++) {
            if (bit_buf[bi]) w |= (1u << i);
        }
        output[word_count++] = w;
    }

    return word_count;
}

// --- Streaming packer for track-level formatting ---

struct BitPacker {
    uint32_t* output;
    int word_count;
    uint32_t current_word;
    int bit_pos;
    int max_words;
};

static void bp_init(BitPacker* bp, uint32_t* buf, int max_words) {
    bp->output = buf;
    bp->word_count = 0;
    bp->current_word = 0;
    bp->bit_pos = 0;
    bp->max_words = max_words;
}

static void bp_emit(BitPacker* bp, uint8_t bit) {
    if (bp->word_count >= bp->max_words) return;
    if (bit) bp->current_word |= (1u << (31 - bp->bit_pos));
    bp->bit_pos++;
    if (bp->bit_pos == 32) {
        bp->output[bp->word_count++] = bp->current_word;
        bp->current_word = 0;
        bp->bit_pos = 0;
    }
}

static void bp_flush(BitPacker* bp) {
    if (bp->bit_pos > 0 && bp->word_count < bp->max_words) {
        bp->output[bp->word_count++] = bp->current_word;
        bp->current_word = 0;
        bp->bit_pos = 0;
    }
}

static int bp_emit_byte_normal(BitPacker* bp, uint8_t byte, int prev_data) {
    for (int i = 7; i >= 0; i--) {
        int data_bit = (byte >> i) & 1;
        if (data_bit) {
            bp_emit(bp, 0); bp_emit(bp, 1);
        } else {
            if (prev_data == 0) {
                bp_emit(bp, 1); bp_emit(bp, 0);
            } else {
                bp_emit(bp, 0); bp_emit(bp, 0);
            }
        }
        prev_data = data_bit;
    }
    return prev_data;
}

static void bp_emit_byte_a1(BitPacker* bp) {
    bp_emit(bp, 0); bp_emit(bp, 1);  // bit7=1
    bp_emit(bp, 0); bp_emit(bp, 0);  // bit6=0 MISSING CLOCK
    bp_emit(bp, 0); bp_emit(bp, 1);  // bit5=1
    bp_emit(bp, 0); bp_emit(bp, 0);  // bit4=0 no clock (prev=1)
    bp_emit(bp, 1); bp_emit(bp, 0);  // bit3=0 clock (prev=0)
    bp_emit(bp, 1); bp_emit(bp, 0);  // bit2=0 clock (prev=0)
    bp_emit(bp, 1); bp_emit(bp, 0);  // bit1=0 clock (prev=0)
    bp_emit(bp, 0); bp_emit(bp, 1);  // bit0=1
}

static int bp_emit_bytes(BitPacker* bp, uint8_t val, int count, int prev) {
    for (int i = 0; i < count; i++)
        prev = bp_emit_byte_normal(bp, val, prev);
    return prev;
}

static uint16_t bp_crc16(const uint8_t* data, int len, uint16_t crc) {
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

int format_track_encode(uint8_t cyl, uint8_t head,
                        uint32_t* output, int max_words) {
    BitPacker bp;
    bp_init(&bp, output, max_words);

    int prev = 0;

    // GAP4a: 80 bytes of 0x4E after the index pulse
    prev = bp_emit_bytes(&bp, 0x4E, 80, prev);

    for (int sec = 1; sec <= 18; sec++) {
        // GAP1: 50 bytes 0x4E
        prev = bp_emit_bytes(&bp, 0x4E, 50, prev);

        // SYNC: 12 bytes 0x00
        prev = bp_emit_bytes(&bp, 0x00, 12, prev);

        // ID AM: 3×A1 (missing clock) + FE
        bp_emit_byte_a1(&bp);
        bp_emit_byte_a1(&bp);
        bp_emit_byte_a1(&bp);
        prev = bp_emit_byte_normal(&bp, 0xFE, 1);

        // ID field: cylinder, head, sector, size code (N=2 for 512 B)
        uint8_t id_field[4] = { cyl, head, (uint8_t)sec, 2 };
        prev = bp_emit_bytes(&bp, cyl,      1, prev);
        prev = bp_emit_bytes(&bp, head,     1, prev);
        prev = bp_emit_bytes(&bp, (uint8_t)sec, 1, prev);
        prev = bp_emit_bytes(&bp, 2,        1, prev);

        // ID CRC over {A1,A1,A1,FE} + id_field
        uint8_t id_presync[4] = { 0xA1, 0xA1, 0xA1, 0xFE };
        uint16_t id_crc = bp_crc16(id_presync, 4, 0xFFFF);
        id_crc = bp_crc16(id_field, 4, id_crc);
        prev = bp_emit_byte_normal(&bp, (id_crc >> 8) & 0xFF, prev);
        prev = bp_emit_byte_normal(&bp, id_crc & 0xFF, (id_crc >> 8) & 1);

        // GAP2: 22 bytes 0x4E
        prev = bp_emit_bytes(&bp, 0x4E, 22, prev);

        // SYNC: 12 bytes 0x00
        prev = bp_emit_bytes(&bp, 0x00, 12, prev);

        // Data AM: 3×A1 + FB
        bp_emit_byte_a1(&bp);
        bp_emit_byte_a1(&bp);
        bp_emit_byte_a1(&bp);
        prev = bp_emit_byte_normal(&bp, 0xFB, 1);

        // 512 data bytes filled with 0xE5 (IBM format fill)
        uint8_t data_presync[4] = { 0xA1, 0xA1, 0xA1, 0xFB };
        uint16_t data_crc = bp_crc16(data_presync, 4, 0xFFFF);
        uint8_t fill_buf[512];
        memset(fill_buf, 0xE5, sizeof(fill_buf));
        data_crc = bp_crc16(fill_buf, 512, data_crc);
        prev = bp_emit_bytes(&bp, 0xE5, 512, prev);
        prev = bp_emit_byte_normal(&bp, (data_crc >> 8) & 0xFF, prev);
        prev = bp_emit_byte_normal(&bp, data_crc & 0xFF, (data_crc >> 8) & 1);

        // GAP3: 66 bytes 0x4E
        prev = bp_emit_bytes(&bp, 0x4E, 66, prev);
    }

    // GAP4b: trailing fill to ensure the write covers the full
    // revolution with margin for rotational speed variation.
    prev = bp_emit_bytes(&bp, 0x4E, 400, prev);

    bp_flush(&bp);
    return bp.word_count;
}
