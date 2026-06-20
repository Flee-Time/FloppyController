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
    emit(1); emit(0);  // bit4=0: clock (prev=1), no data
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
    emit_byte_normal(crc & 0xFF, (crc >> 7) & 1);

    // Pad to fill any partial 32-bit word at the end.
    while (bit_count & 31) emit(0);

    // Pack bits into 32-bit words, MSB first within each word.
    int word_count = 0;
    int bi = 0;
    while (bi < bit_count) {
        uint32_t w = 0;
        for (int i = 0; i < 32 && bi < bit_count; i++, bi++) {
            if (bit_buf[bi]) w |= (1u << (31 - i));
        }
        output[word_count++] = w;
    }

    return word_count;
}
