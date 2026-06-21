#include "tusb.h"
#include "debug_serial.h"
#include <string.h>
#include <stdio.h>

static char rx_ring[64];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

static void debug_putc(char c) {
    if (!tud_cdc_connected()) return;
    tud_cdc_write_char(c);
}

static void debug_write_str(const char* str) {
    if (!tud_cdc_connected()) return;
    uint32_t len = strlen(str);
    uint32_t written = 0;
    while (written < len) {
        uint32_t avail = tud_cdc_write_available();
        if (avail == 0) { tud_cdc_write_flush(); continue; }
        uint32_t chunk = len - written;
        if (chunk > avail) chunk = avail;
        tud_cdc_write(str + written, chunk);
        written += chunk;
        tud_cdc_write_flush();
    }
}

void debug_serial_init(void) {
}

void debug_serial_write(const char* str) {
    debug_write_str(str);
}

void debug_serial_write_hex32(uint32_t val) {
    char buf[11];
    int len = 0;
    if (val == 0) {
        buf[len++] = '0';
    } else {
        char tmp[8];
        int t = 0;
        while (val && t < 8) {
            uint8_t nibble = val & 0xF;
            tmp[t++] = nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
            val >>= 4;
        }
        while (t > 0) buf[len++] = tmp[--t];
    }
    buf[len] = '\0';
    debug_write_str(buf);
}

void debug_serial_write_dec32(uint32_t val) {
    char buf[12];
    int len = 0;
    if (val == 0) {
        buf[len++] = '0';
    } else {
        char tmp[10];
        int t = 0;
        while (val && t < 10) {
            tmp[t++] = '0' + (val % 10);
            val /= 10;
        }
        while (t > 0) buf[len++] = tmp[--t];
    }
    buf[len] = '\0';
    debug_write_str(buf);
}

void debug_serial_flush(void) {
    if (tud_cdc_connected()) {
        tud_cdc_write_flush();
    }
}

char debug_serial_read(void) {
    if (rx_head == rx_tail) return 0;
    char c = rx_ring[rx_tail];
    rx_tail = (rx_tail + 1) % sizeof(rx_ring);
    return c;
}

// --- TinyUSB CDC Callbacks ---
extern "C" {

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void) itf;
    (void) dtr;
    (void) rts;
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const* coding) {
    (void) itf;
    (void) coding;
}

void tud_cdc_rx_cb(uint8_t itf) {
    (void) itf;
    // Don't consume data here — let the main loop read it directly.
    // The ring buffer is only for debug serial command mode.
}
}
