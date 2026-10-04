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
#include "gw/gw_proto.h"
#include "usb_task.h"
#include "usb_msc.h"

volatile bool core1_request_pending = false;
volatile bool core1_write_request = false;
volatile bool core1_format_request = false;
volatile uint32_t shared_target_lba = 0;
volatile bool core1_result_ready = false;
volatile bool core1_result_success = false;
volatile bool core1_format_done = false;
volatile bool core1_format_success = false;
volatile bool format_in_progress = false;
volatile uint8_t shared_sector_buffer[512];
volatile uint8_t shared_format_cyl = 0;
volatile uint8_t shared_format_head = 0;
volatile bool force_recalibrate = false;
volatile bool sense_media_changed = true;
volatile bool disk_present = false;
volatile bool scsi_format_requested = false;
volatile uint8_t shared_debug_state = 0;
volatile uint32_t shared_debug_pll = 0;
volatile uint8_t shared_read_progress = 0;
volatile uint8_t shared_last_cyl = 0;
volatile uint8_t shared_last_head = 0;
volatile uint8_t shared_last_sec = 0;
volatile IoStage shared_io_stage = IO_IDLE;

void core1_floppy_worker() {
    init_hardware();

    uint32_t last_activity = to_ms_since_boot(get_absolute_time());
    bool drive_active = false;

    static uint8_t  cache_cyl = 0xFF, cache_head = 0xFF, cache_sec = 0;
    static uint8_t  cached_sector[512];
    static bool     cache_valid = false;

    disk_present = false;
    sense_media_changed = true;
    poll_disk_change(to_ms_since_boot(get_absolute_time()));

    while (true) {
        uint32_t now = to_ms_since_boot(get_absolute_time());

        if (core1_request_pending) {
            __dmb();
            bool is_write = core1_write_request;
            if (!drive_active) {
                shared_io_stage = IO_SPINUP;
                drive_select(true);
                drive_motor(true);
                set_density(true);
                busy_wait_ms(500);
                drive_active = true;
            }
            last_activity = now;

            uint8_t cyl, head, sec;
            lba_to_chs(shared_target_lba, &cyl, &head, &sec);

            shared_io_stage = IO_SEEK;
            seek_physical_track(cyl, head);

            bool ok;
            if (is_write) {
                ok = write_physical_sector(cyl, head, sec, (const uint8_t*)shared_sector_buffer);
                cache_valid = false;
            } else if (cache_valid && cache_cyl == cyl && cache_head == head && cache_sec == sec) {
                memcpy((uint8_t*)shared_sector_buffer, cached_sector, 512);
                ok = true;
            } else {
                shared_io_stage = IO_READ;
                ok = read_physical_sector(cyl, head, sec, (uint8_t*)shared_sector_buffer);
                if (!ok) {
                    seek_physical_track(cyl, head);
                    ok = read_physical_sector(cyl, head, sec, (uint8_t*)shared_sector_buffer);
                }
                if (!ok) {
                    if (cyl > 0) {
                        seek_physical_track(cyl - 1, head);
                    } else {
                        seek_physical_track(cyl + 1, head);
                    }
                    seek_physical_track(cyl, head);
                    ok = read_physical_sector(cyl, head, sec, (uint8_t*)shared_sector_buffer);
                }
            }

            if (ok && !is_write) {
                cache_cyl = cyl; cache_head = head; cache_sec = sec;
                memcpy(cached_sector, (const uint8_t*)shared_sector_buffer, 512);
                cache_valid = true;
            }

            core1_result_success = ok;
            shared_io_stage = IO_DONE;
            core1_write_request = false;
            __dmb();
            core1_result_ready = true;
            __dmb();
            core1_request_pending = false;
        }

        if (core1_format_request && !core1_request_pending) {
            __dmb();
            if (!drive_active) {
                drive_select(true);
                drive_motor(true);
                set_density(true);
                shared_io_stage = IO_SPINUP;
                busy_wait_ms(500);
                drive_active = true;
            }
            last_activity = now;

            uint8_t fcyl = shared_format_cyl;
            uint8_t fhead = shared_format_head;

            shared_io_stage = IO_SEEK;
            seek_physical_track(fcyl, fhead);
            bool fmt_ok = format_physical_track(fcyl, fhead);
            cache_valid = false;
            core1_format_success = fmt_ok;

            __dmb();
            core1_format_done = true;
            __dmb();
            core1_format_request = false;
        }

        if (drive_active && !core1_request_pending && (now - last_activity > 2000)) {
            drive_select(false);
            drive_motor(false);
            drive_active = false;
            cache_valid = false;
        }

        if (!drive_active && !core1_request_pending) {
            static uint32_t last_poll = 0;
            if (now - last_poll > 500) {
                poll_disk_change(now);
                last_poll = now;
            }
        }

        // This core owns floppy timing. Do not depend on core 0's alarm
        // pool/IRQ to wake it while core 0 is servicing sustained USB I/O.
        busy_wait_us_32(10);
    }
}

int main() {
    set_sys_clock_khz(SYS_CLOCK_KHZ, true);

    led_init();
    led_set_rgb(0, 0, 0);

    mode_config_init();

    if (!mode_has_gw()) {
        multicore_launch_core1(core1_floppy_worker);
    } else {
        init_hardware();
    }

    tusb_init();

    if (mode_has_debug()) {
        debug_serial_init();
    }
    if (mode_has_gw()) {
        gw_proto_init();
    }

    enum { FMT_IDLE, FMT_STARTING, FMT_WAITING } fmt_state = FMT_IDLE;
    int fmt_cyl = 0, fmt_head = 0;
    uint32_t fmt_track_timeout = 0;

    while (true) {
        usb_task_run();
        usb_msc_poll();

        if (mode_has_debug()) {
            debug_serial_flush();
        }

        if (mode_has_gw()) {
            gw_proto_poll();
        }

        // --- Format state machine (non-blocking) ---
        if (scsi_format_requested && fmt_state == FMT_IDLE) {
            if (gpio_get(PIN_WP) == 0) {
                debug_serial_write("[E] FORMAT aborted: disk is write-protected\r\n");
                scsi_format_requested = false;
            } else if (!mode_has_write()) {
                debug_serial_write("[E] FORMAT aborted: write-enable DIP not set\r\n");
                scsi_format_requested = false;
            } else {
                debug_serial_write("[I] FORMAT start\r\n");
                fmt_state = FMT_STARTING;
                format_in_progress = true;
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
            if (core1_format_done && !core1_format_request) {
                __dmb();
                if (!core1_format_success) {
                    debug_serial_write("[E] FORMAT write/verify failed\r\n");
                    sense_media_changed = true;
                    format_in_progress = false;
                    fmt_state = FMT_IDLE;
                    continue;
                }
                if (mode_has_debug()) {
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
                    force_recalibrate = true;
                    disk_present = true;
                    sense_media_changed = true;
                    fmt_state = FMT_IDLE;
                    format_in_progress = false;
                } else {
                    fmt_state = FMT_STARTING;
                }
            } else if (to_ms_since_boot(get_absolute_time()) - fmt_track_timeout > 10000) {
                if (mode_has_debug())
                    debug_serial_write("[E] FORMAT timeout\r\n");
                // Leave the outstanding request owned by core 1 until it exits.
                sense_media_changed = true;
                fmt_state = FMT_IDLE;
                format_in_progress = false;
            }
        }

    }
    return 0;
}
