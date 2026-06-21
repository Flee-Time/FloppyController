#include "pico/stdlib.h"
#include "hardware/pio.h"

#include "mfm_decoder.h"
#include "floppy_hw.h"
#include "shared_state.h"

void lba_to_chs(uint32_t lba, uint8_t *cylinder, uint8_t *head, uint8_t *sector) {
    *sector   = (lba % 18) + 1;
    *head     = (lba / 18) % 2;
    *cylinder = (lba / (18 * 2));
}

static uint16_t crc16_ccitt(const uint8_t* data, int len, uint16_t crc = 0xFFFF) {
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

bool read_physical_sector(uint8_t target_cyl, uint8_t target_head, uint8_t target_sec, uint8_t* buffer) {
    pio_sm_clear_fifos(flux_pio, flux_sm);

    uint32_t shift_reg = 0;
    int bit_count = 0;
    bool in_sync = false;

    enum State { HUNT_ID, READ_ID, HUNT_DATA, READ_DATA };
    State state = HUNT_ID;
    int byte_index = 0;
    uint8_t sector_header[6];
    uint8_t data_crc_bytes[2];

    static uint32_t s_pll = 200;

    shared_debug_state = 0;
    shared_debug_pll = s_pll;
    shared_read_progress = 0;

    uint32_t start_time = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - start_time < 500) {
        if (pio_sm_is_rx_fifo_empty(flux_pio, flux_sm)) continue;

        uint32_t pio_val = pio_sm_get(flux_pio, flux_sm);
        uint32_t ticks = 0xFFFFFFFF - pio_val;

        if (ticks < 100 || ticks > 700) {
            in_sync = false;
            bit_count = 0;
            shift_reg = 0;
            continue;
        }

        if (ticks > (s_pll - 40) && ticks < (s_pll + 40)) {
            s_pll = (s_pll * 15 + ticks) / 16;
        } else if (ticks > (s_pll / 2 - 30) && ticks < (s_pll / 2 + 30)) {
            s_pll = (s_pll * 15 + ticks * 2) / 16;
        }

        uint32_t threshold_short_med = (s_pll * 5) / 4;
        uint32_t threshold_med_long  = (s_pll * 7) / 4;

        uint8_t windows = 4;
        if (ticks < threshold_short_med) windows = 2;
        else if (ticks < threshold_med_long) windows = 3;

        for (int i = 0; i < windows; i++) {
            shift_reg = (shift_reg << 1) | (i == windows - 1 ? 1 : 0);
            bit_count++;

            if ((shift_reg & 0xFFFF) == 0x4489) {
                in_sync = true;
                bit_count = 0;
                if (shared_read_progress < 1) shared_read_progress = 1;
                continue;
            }

                if (in_sync && bit_count == 16) {
                    uint8_t data_byte = 0;
                    if (shift_reg & 0x4000) data_byte |= 0x80;
                    if (shift_reg & 0x1000) data_byte |= 0x40;
                    if (shift_reg & 0x0400) data_byte |= 0x20;
                    if (shift_reg & 0x0100) data_byte |= 0x10;
                    if (shift_reg & 0x0040) data_byte |= 0x08;
                    if (shift_reg & 0x0010) data_byte |= 0x04;
                    if (shift_reg & 0x0004) data_byte |= 0x02;
                    if (shift_reg & 0x0001) data_byte |= 0x01;

                    bit_count = 0;

                if (state == HUNT_ID) {
                    if (data_byte == 0xFE) { state = READ_ID; byte_index = 0; }
                } else if (state == READ_ID) {
                    sector_header[byte_index++] = data_byte;
                    if (byte_index == 6) {
                        static const uint8_t id_presync[4] = {0xA1, 0xA1, 0xA1, 0xFE};
                        uint16_t crc = crc16_ccitt(id_presync, 4);
                        crc = crc16_ccitt(sector_header, 4, crc);
                        uint16_t want_crc = (sector_header[4] << 8) | sector_header[5];

                        if (crc == want_crc && shared_read_progress < 2)
                            shared_read_progress = 2;

                        shared_last_cyl  = sector_header[0];
                        shared_last_head = sector_header[1];
                        shared_last_sec  = sector_header[2];

                        if (crc == want_crc &&
                            sector_header[0] == target_cyl &&
                            sector_header[1] == target_head &&
                            sector_header[2] == target_sec) {
                            state = HUNT_DATA;
                            in_sync = false; shift_reg = 0; bit_count = 0;
                            shared_read_progress = 3;
                        } else {
                            state = HUNT_ID; in_sync = false;
                        }
                    }
                } else if (state == HUNT_DATA) {
                    if (data_byte == 0xFB) {
                        state = READ_DATA; byte_index = 0;
                        if (shared_read_progress < 4) shared_read_progress = 4;
                    }
                } else if (state == READ_DATA) {
                    if (byte_index < 512) {
                        buffer[byte_index] = data_byte;
                    } else {
                        data_crc_bytes[byte_index - 512] = data_byte;
                    }
                    byte_index++;

                    if (byte_index == 514) {
                        static const uint8_t data_presync[4] = {0xA1, 0xA1, 0xA1, 0xFB};
                        uint16_t crc = crc16_ccitt(data_presync, 4);
                        crc = crc16_ccitt(buffer, 512, crc);
                        uint16_t want_crc = (data_crc_bytes[0] << 8) | data_crc_bytes[1];

                        if (crc == want_crc) {
                            shared_read_progress = 5;
                            shared_debug_pll = s_pll;
                            return true;
                        } else {
                            state = HUNT_ID; in_sync = false; bit_count = 0; shift_reg = 0;
                            shared_debug_state = 3;
                        }
                    }
                }
            }
        }
    }

    shared_debug_state = 4;
    shared_debug_pll = s_pll;
    return false;
}
