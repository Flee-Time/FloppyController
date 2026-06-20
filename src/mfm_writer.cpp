#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"

#include "mfm_writer.h"
#include "mfm_encoder.h"
#include "floppy_hw.h"
#include "hw_config.h"
#include "shared_state.h"

bool write_physical_sector(uint8_t target_cyl, uint8_t target_head,
                           uint8_t target_sec, const uint8_t* data) {
    // 1.  Encode the sector data into the MFM half-cell bitstream.
    static uint32_t encode_buf[320];
    int words = mfm_encode_sector(data, encode_buf);

    // 2.  Pre-load the write PIO TX FIFO.
    PIO wpio = pio1;
    uint wsm = write_sm;
    pio_sm_set_enabled(wpio, wsm, false);
    pio_sm_clear_fifos(wpio, wsm);
    pio_sm_restart(wpio, wsm);

    for (int i = 0; i < words && i < 8; i++)
        pio_sm_put(wpio, wsm, encode_buf[i]);

    // 3.  Find the target sector on the track.
    pio_sm_clear_fifos(flux_pio, flux_sm);

    uint32_t shift_reg = 0;
    int bit_count = 0;
    bool in_sync = false;
    int byte_index = 0;
    uint8_t sector_header[6];
    bool sector_found = false;

    static uint32_t s_pll = 200;

    enum State { HUNT_ID, READ_ID };
    State state = HUNT_ID;

    uint32_t start_time = to_ms_since_boot(get_absolute_time());

    while (!sector_found &&
           to_ms_since_boot(get_absolute_time()) - start_time < 1000) {
        if (pio_sm_is_rx_fifo_empty(flux_pio, flux_sm)) continue;

        uint32_t pio_val = pio_sm_get(flux_pio, flux_sm);
        uint32_t ticks = 0xFFFFFFFF - pio_val;

        if (ticks < 100 || ticks > 700) {
            in_sync = false; bit_count = 0; shift_reg = 0; continue;
        }

        if (ticks > (s_pll - 40) && ticks < (s_pll + 40))
            s_pll = (s_pll * 15 + ticks) / 16;
        else if (ticks > (s_pll / 2 - 30) && ticks < (s_pll / 2 + 30))
            s_pll = (s_pll * 15 + ticks * 2) / 16;

        uint32_t th_sm = (s_pll * 5) / 4, th_ml = (s_pll * 7) / 4;
        uint8_t windows = 4;
        if (ticks < th_sm) windows = 2;
        else if (ticks < th_ml) windows = 3;

        for (int i = 0; i < windows && !sector_found; i++) {
            shift_reg = (shift_reg << 1) | (i == windows - 1 ? 1 : 0);
            bit_count++;

            if ((shift_reg & 0xFFFF) == 0x4489) {
                in_sync = true; bit_count = 0; continue;
            }

            if (in_sync && bit_count == 16) {
                uint8_t db = 0;
                if (shift_reg & 0x4000) db |= 0x80;
                if (shift_reg & 0x1000) db |= 0x40;
                if (shift_reg & 0x0400) db |= 0x20;
                if (shift_reg & 0x0100) db |= 0x10;
                if (shift_reg & 0x0040) db |= 0x08;
                if (shift_reg & 0x0010) db |= 0x04;
                if (shift_reg & 0x0004) db |= 0x02;
                if (shift_reg & 0x0001) db |= 0x01;
                bit_count = 0;

                if (state == HUNT_ID) {
                    if (db == 0xFE) { state = READ_ID; byte_index = 0; }
                } else if (state == READ_ID) {
                    sector_header[byte_index++] = db;
                    if (byte_index == 6) {
                        if (sector_header[0] == target_cyl &&
                            sector_header[1] == target_head &&
                            sector_header[2] == target_sec) {
                            sector_found = true;
                        } else {
                            state = HUNT_ID; in_sync = false;
                        }
                    }
                }
            }
        }
    }

    if (!sector_found) return false;

    // 4.  Assert WGATE, wait for write current to stabilise, then start
    //     the write PIO and feed the remaining bitstream.
    write_gate(true);
    sleep_us(8);
    pio_sm_set_enabled(wpio, wsm, true);

    int next = 8;
    while (next < words) {
        if (!pio_sm_is_tx_fifo_full(wpio, wsm)) {
            pio_sm_put(wpio, wsm, encode_buf[next++]);
        }
    }

    // 5.  Wait for the PIO to finish, then immediately kill write gate.
    //     The last word encodes the CRC + minimal padding — WGATE must
    //     drop before the gap ends so the next sector isn't erased.
    while (!pio_sm_is_tx_fifo_empty(wpio, wsm)) tight_loop_contents();
    sleep_us(10);
    pio_sm_set_enabled(wpio, wsm, false);
    write_gate(false);

    return true;
}
