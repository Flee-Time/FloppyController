#pragma once
#include <stddef.h>
#include <stdint.h>

// GreaseWeasel flux encoding.
// Flux durations T are encoded as:
//   1..249:          1 byte  (T itself)
//   250..1524:       2 bytes (250 + (T-250)/255, 1 + (T-250)%255)
//   1525..2^28-1:    6 bytes (255, Space opcode, 28-bit value)
//
// Stream is terminated with a 0x00 byte.

enum { GP_CUT1 = 250, GP_CUT2 = 1525, GP_CUT6 = (1 << 28) - 1 };

static inline uint8_t* greasepack(uint8_t* buf, uint8_t* end, unsigned value) {
    if (!buf || buf == end) return buf;
    size_t left = (size_t)(end - buf);
    size_t need = value < GP_CUT1 ? 1 : value < GP_CUT2 ? 2 : 6;
    if (need > left) { *buf = 0; return end; }

    if (value < GP_CUT1) {
        *buf++ = (uint8_t)value;
    } else if (value < GP_CUT2) {
        unsigned high = (value - 250) / 255;
        *buf++ = (uint8_t)(250 + high);
        *buf++ = (uint8_t)(1 + (value - 250) % 255);
    } else {
        if (value > GP_CUT6) value = GP_CUT6;
        *buf++ = 255;
        *buf++ = 2;
        *buf++ = (uint8_t)(1 | ((value << 1) & 255));
        *buf++ = (uint8_t)(1 | ((value >> 6) & 255));
        *buf++ = (uint8_t)(1 | ((value >> 13) & 255));
        *buf++ = (uint8_t)(1 | ((value >> 20) & 255));
    }
    return buf;
}

static inline unsigned greaseunpack(uint8_t** buf_, uint8_t* end,
                                    bool is_gw) {
#define BUF (*buf_)
    if (!is_gw) {
        if (!BUF || BUF == end) return 0xffff;
        return *BUF++;
    }
    while (true) {
        if (!BUF || BUF == end) return 0xffff;
        size_t left = (size_t)(end - BUF);
        uint8_t data = *BUF++;
        size_t need = data == 255 ? 6 : data >= GP_CUT1 ? 2 : 1;
        if (left < need) { BUF = end; return 0xffff; }
        if (need == 1) return data;
        if (need == 2) {
            uint8_t data2 = *BUF++;
            return (data - GP_CUT1) * 255 + (data2 - 1) + 250;
        }
        uint8_t data2 = *BUF++;
        if (data2 != 2) { BUF += 4; continue; }
        uint32_t val = (*BUF++ & 254) >> 1;
        val += (*BUF++ & 254) << 6;
        val += (*BUF++ & 254) << 13;
        val += (*BUF++ & 254) << 20;
        return val;
    }
#undef BUF
}
