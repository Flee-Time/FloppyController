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
volatile uint32_t shared_target_lba = 0;
volatile bool core1_result_ready = false;
volatile bool core1_result_success = false;
uint8_t shared_sector_buffer[512];
volatile bool sense_media_changed = true;
volatile bool disk_present = false;
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

        if (drive_active && !core1_request_pending && (now - last_activity > 2000)) {
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

    while (true) {
        tud_task();
        if (mode_has(MODE_DEBUG_SERIAL)) {
            debug_serial_flush();
        }
    }
    return 0;
}
