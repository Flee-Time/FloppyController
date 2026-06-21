#pragma GCC optimize ("O3")

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "tusb.h"

#include "shared_state.h"
#include "hw_config.h"
#include "led_drv.h"
#include "floppy_hw.h"
#include "mfm_decoder.h"
#include "mfm_writer.h"
#include "mode_config.h"
#include "debug_serial.h"

volatile bool core1_request_pending = false;
volatile bool core1_write_request = false;
volatile bool core1_format_request = false;
volatile uint32_t shared_target_lba = 0;
volatile bool core1_result_ready = false;
volatile bool core1_result_success = false;
volatile bool core1_format_done = false;
uint8_t shared_sector_buffer[512];
volatile uint8_t shared_format_cyl = 0;
volatile uint8_t shared_format_head = 0;
volatile bool shared_fmt_rd_ok = false;
volatile bool sense_media_changed = true;
volatile bool disk_present = false;
volatile bool scsi_format_requested = false;
volatile bool gw_busy = false;
volatile uint8_t shared_debug_state = 0;
volatile uint32_t shared_debug_pll = 0;
volatile uint32_t shared_ticks_sample[8];
volatile uint8_t shared_ticks_count = 0;
volatile uint8_t shared_read_progress = 0;
volatile uint8_t shared_last_cyl = 0;
volatile uint8_t shared_last_head = 0;
volatile uint8_t shared_last_sec = 0;

void core1_floppy_worker() {
    init_hardware();

    uint32_t last_activity = to_ms_since_boot(get_absolute_time());
    bool drive_active = false;

    disk_present = false;
    sense_media_changed = true;

    poll_disk_change(to_ms_since_boot(get_absolute_time()));

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (core1_request_pending) {
            if (!drive_active) {
                drive_select(true);
                drive_motor(true);
                sleep_ms(500);
                drive_active = true;
            }
            last_activity = now;

            uint8_t cyl, head, sec;
            lba_to_chs(shared_target_lba, &cyl, &head, &sec);

            seek_physical_track(cyl, head);

            bool ok;
            if (core1_write_request) {
                ok = write_physical_sector(cyl, head, sec, shared_sector_buffer);
            } else {
                ok = read_physical_sector(cyl, head, sec, shared_sector_buffer);
                if (!ok) {
                    seek_physical_track(cyl, head);
                    ok = read_physical_sector(cyl, head, sec, shared_sector_buffer);
                }
            }

            core1_result_success = ok;
            __dmb();
            core1_request_pending = false;
            core1_result_ready = true;
        }

        if (core1_format_request && !core1_request_pending) {
            if (!drive_active) {
                drive_select(true);
                drive_motor(true);
                sleep_ms(500);
                drive_active = true;
            }
            last_activity = now;

            uint8_t fcyl = shared_format_cyl;
            uint8_t fhead = shared_format_head;

            seek_physical_track(fcyl, fhead);
            bool fmt_ok = format_physical_track(fcyl, fhead);

            __dmb();
            core1_format_request = false;
            core1_format_done = true;
            (void)fmt_ok;
        }

        if (drive_active && !core1_request_pending && !gw_busy && (now - last_activity > 2000)) {
            drive_select(false);
            drive_motor(false);
            drive_active = false;
        }

        if (!drive_active && !core1_request_pending) {
            static uint32_t last_poll = 0;
            if (now - last_poll > 500) {
                poll_disk_change(now);
                last_poll = now;
            }
        }

        sleep_us(10);
    }
}

// --- GreaseWeasel raw flux capture ---

static void gw_send_u32_hex(uint32_t v) {
    char buf[9];
    for (int i = 7; i >= 0; i--) {
        uint8_t nib = (v >> (i * 4)) & 0xF;
        buf[7 - i] = nib < 10 ? '0' + nib : 'A' + nib - 10;
    }
    buf[8] = '\n';
    for (int i = 0; i < 9; i++) {
        while (!tud_cdc_connected()) { tud_task(); }
        while (!tud_cdc_write_available()) { tud_task(); tud_cdc_write_flush(); }
        tud_cdc_write_char(buf[i]);
    }
    tud_cdc_write_flush();
}

static void gw_str(const char* s) {
    while (!tud_cdc_connected()) { tud_task(); }
    while (*s) {
        while (!tud_cdc_write_available()) { tud_task(); tud_cdc_write_flush(); }
        tud_cdc_write_char(*s++);
    }
    tud_cdc_write_flush();
}

static void gw_read_track(uint8_t cyl, uint8_t head) {
    gw_busy = true;
    __dmb();
    drive_select(true);
    drive_motor(true);
    sleep_ms(500);

    tud_task();
    gw_str("SEEK\r\n");

    seek_physical_track(cyl, head);

    tud_task();
    gw_str("WAIT_INDEX\r\n");

    // Wait for index pulse to sync capture start
    // Stop the flux PIO, wait for INDEX edge, restart for clean start
    pio_sm_set_enabled(flux_pio, flux_sm, false);
    {
        int timeout = 0;
        while (gpio_get(PIN_INDEX) == 1) { sleep_us(50); if (++timeout > 20000) break; }
        while (gpio_get(PIN_INDEX) == 0) { sleep_us(50); if (++timeout > 20000) break; }
    }
    pio_sm_restart(flux_pio, flux_sm);
    pio_sm_clear_fifos(flux_pio, flux_sm);
    pio_sm_set_enabled(flux_pio, flux_sm, true);

    tud_task();
    gw_str("FLUX\r\n");

    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint32_t count = 0;

    while (to_ms_since_boot(get_absolute_time()) - start < 1000) {
        tud_task();
        if (pio_sm_is_rx_fifo_empty(flux_pio, flux_sm)) continue;
        uint32_t v = pio_sm_get(flux_pio, flux_sm);
        uint32_t tk = 0xFFFFFFFF - v;
        gw_send_u32_hex(tk);
        count++;
        if (count >= 50000) break;
    }

    gw_str("DONE\r\n");
    gw_busy = false;
    (void)count;
}

int main() {
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    led_init();
    led_set_rgb(0, 0, 0);

    mode_config_init();
    uint8_t mode = mode_config_get();

    multicore_launch_core1(core1_floppy_worker);

    tusb_init();

    if (mode_has(MODE_DEBUG_SERIAL)) {
        debug_serial_init();
    }

    enum { FMT_IDLE, FMT_STARTING, FMT_WAITING } fmt_state = FMT_IDLE;
    int fmt_cyl = 0, fmt_head = 0;
    uint32_t fmt_track_timeout = 0;

    while (true) {
        tud_task();

        if (mode_has(MODE_DEBUG_SERIAL)) {
            debug_serial_flush();
        }

        if (mode_has(MODE_GREASEWEASEL)) {
            char cmd = debug_serial_read();
            if (cmd == 'R' || cmd == 'r') {
                debug_serial_write("[I] GW: reading raw flux\r\n");
                uint8_t tc = (current_track >= 0) ? (uint8_t)current_track : 0;
                gw_read_track(tc, 0);
            } else if (cmd == 'F' || cmd == 'f') {
                scsi_format_requested = true;
            }
        }

        // --- Format state machine (non-blocking) ---
        if (scsi_format_requested && fmt_state == FMT_IDLE) {
            if (gpio_get(PIN_WP) == 0) {
                debug_serial_write("[E] FORMAT aborted: disk is write-protected\r\n");
                scsi_format_requested = false;
            } else if (!mode_has(MODE_WRITE_ENABLE)) {
                debug_serial_write("[E] FORMAT aborted: write-enable DIP not set\r\n");
                scsi_format_requested = false;
            } else {
                debug_serial_write("[I] FORMAT start\r\n");
                fmt_state = FMT_STARTING;
                fmt_cyl = 0;
                fmt_head = 0;
            }
            scsi_format_requested = false;
        }

        if (fmt_state == FMT_STARTING) {
            shared_format_cyl = (uint8_t)fmt_cyl;
            shared_format_head = (uint8_t)fmt_head;
            core1_format_done = false;
            __dmb();
            core1_format_request = true;
            fmt_track_timeout = to_ms_since_boot(get_absolute_time());
            fmt_state = FMT_WAITING;
        }

        if (fmt_state == FMT_WAITING) {
            if (core1_format_done) {
                if (mode_has(MODE_DEBUG_SERIAL)) {
                    debug_serial_write("[I] FORMAT cyl=");
                    debug_serial_write_dec32(fmt_cyl);
                    debug_serial_write(" head=");
                    debug_serial_write_dec32(fmt_head);
                    debug_serial_write("\r\n");
                }
                fmt_head++;
                if (fmt_head >= 2) { fmt_head = 0; fmt_cyl++; }
                if (fmt_cyl >= 80) {
                    debug_serial_write("[I] FORMAT done\r\n");
                    current_track = -1;
                    disk_present = true;
                    sense_media_changed = true;
                    fmt_state = FMT_IDLE;
                } else {
                    fmt_state = FMT_STARTING;
                }
            } else if (to_ms_since_boot(get_absolute_time()) - fmt_track_timeout > 10000) {
                if (mode_has(MODE_DEBUG_SERIAL))
                    debug_serial_write("[E] FORMAT timeout\r\n");
                core1_format_request = false;
                disk_present = true;
                sense_media_changed = true;
                fmt_state = FMT_IDLE;
            }
        }

    }
    return 0;
}
