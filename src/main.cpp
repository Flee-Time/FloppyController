#pragma GCC optimize ("O3")

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "tusb.h"

// Include our custom modules
#include "shared_state.h"
#include "hw_config.h"
#include "led_drv.h"
#include "floppy_hw.h"
#include "mfm_decoder.h"

// --- DEFINE the shared variables here ---
volatile bool core1_request_pending = false;
volatile uint32_t shared_target_lba = 0;
volatile bool core1_result_ready = false;
volatile bool core1_result_success = false;
volatile bool core1_needs_calibration = false;
uint8_t shared_sector_buffer[512];
volatile bool sense_media_changed = true;
volatile bool disk_present = false;
// ----------------------------------------

void core1_floppy_worker() {
    led_init(); 
    init_hardware();

    uint32_t last_activity = to_ms_since_boot(get_absolute_time());
    uint32_t last_poll = last_activity;
    bool drive_active = false;

    // Boot assuming empty until polled
    disk_present = false; 
    sense_media_changed = true;

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        // --- 1. Process USB Read Requests ---
        if (core1_request_pending) {
            if (!drive_active) {
                drive_select(true);
                drive_motor(true);
                drive_active = true;
                sleep_ms(800); 
            }
            last_activity = now;
            
            uint8_t cyl, head, sec;
            lba_to_chs(shared_target_lba, &cyl, &head, &sec);
            
            seek_physical_track(cyl, head);
            core1_result_success = read_physical_sector(cyl, head, sec, shared_sector_buffer);
            
            core1_request_pending = false;
            core1_result_ready = true;
        }

        // --- 2. Motor Spindown Timer ---
        if (drive_active && !core1_request_pending && (now - last_activity > 2000)) {
            drive_select(false);
            drive_motor(false);
            drive_active = false;
        }

        // --- 3. Passive Hardware Polling ---
        // Runs every 500ms when idle. Delegates to poll_disk_change(), which
        // knows how to deal with the latched DSKCHG signal correctly (see
        // floppy_hw.cpp) -- a plain pin read can't tell new media has
        // arrived, only that something changed.
        if (!drive_active && !core1_request_pending && (now - last_poll > 500)) {
            poll_disk_change(now);
            last_poll = now;
        }

        sleep_us(100); 
    }
}

int main() {
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);
    
    // Boot Core 1
    multicore_launch_core1(core1_floppy_worker);

    // Boot Core 0 USB
    tusb_init();

    while (true) {
        tud_task(); 
    }
    return 0;
}
