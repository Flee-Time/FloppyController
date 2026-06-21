#include "tusb.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "shared_state.h"
#include "mode_config.h"
#include "debug_serial.h"
#include "led_drv.h"
#include "hw_config.h"
#include "floppy_hw.h"
#include <string.h>

static bool format_in_progress = false;

extern "C" {
    bool tud_msc_test_unit_ready_cb(uint8_t lun) {
        (void) lun;

        if (core1_format_request) {
            if (mode_has(MODE_DEBUG_SERIAL))
                debug_serial_write("[I] TUR: format in progress\r\n");
            tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x04);
            return false;
        }

        if (!disk_present) {
            if (mode_has(MODE_DEBUG_SERIAL))
                debug_serial_write("[I] TUR: no disk\r\n");
            tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
            return false;
        }

        if (sense_media_changed) {
            if (mode_has(MODE_DEBUG_SERIAL))
                debug_serial_write("[I] TUR: media changed\r\n");
            sense_media_changed = false;
            tud_msc_set_sense(lun, SCSI_SENSE_UNIT_ATTENTION, 0x28, 0x00);
            return false;
        }
        return true;
    }

    void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size) {
        (void) lun; *block_count = 2880; *block_size = 512;
    }

    void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4]) {
        (void) lun;
        const char vid[] = "FleeTime"; const char pid[] = "Floppy Drive"; const char rev[] = "2.0";
        memcpy(vendor_id  , vid, strlen(vid));
        memcpy(product_id , pid, strlen(pid));
        memcpy(product_rev, rev, strlen(rev));
    }

    int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
        (void) lun; (void) offset;
        uint32_t block_count = bufsize / 512;
        uint8_t* ptr = (uint8_t*) buffer;

        if (mode_has(MODE_DEBUG_SERIAL)) {
            debug_serial_write("[I] READ lba=");
            debug_serial_write_dec32(lba);
            debug_serial_write(" n=");
            debug_serial_write_dec32(block_count);
            debug_serial_write("\r\n");
        }

        for (uint32_t i = 0; i < block_count; i++) {
            shared_target_lba = lba + i;
            core1_result_ready = false;

            __dmb();
            core1_request_pending = true;

            uint32_t wait_start = to_ms_since_boot(get_absolute_time());
            while (!core1_result_ready) {
                if (to_ms_since_boot(get_absolute_time()) - wait_start > 5000) {
                    if (mode_has(MODE_DEBUG_SERIAL))
                        debug_serial_write("[E] READ timeout\r\n");
                    tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
                    core1_request_pending = false;
                    return -1;
                }
                tight_loop_contents();
            }

            __dmb();

            if (!core1_result_success) {
                led_set_rgb(50, 0, 0);
                sleep_ms(80);
                led_set_rgb(0, 0, 0);

                if (mode_has(MODE_DEBUG_SERIAL)) {
                    debug_serial_write("[W] READ fail lba=");
                    debug_serial_write_dec32(lba + i);
                    debug_serial_write(" pr=");
                    debug_serial_write_dec32(shared_read_progress);
                    debug_serial_write(" pll=");
                    debug_serial_write_dec32(shared_debug_pll);
                    debug_serial_write(" hdr=");
                    debug_serial_write_dec32(shared_last_cyl);
                    debug_serial_write("/");
                    debug_serial_write_dec32(shared_last_head);
                    debug_serial_write("/");
                    debug_serial_write_dec32(shared_last_sec);
                    debug_serial_write("\r\n");
                }
                tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
                return -1;
            }

            memcpy(ptr, shared_sector_buffer, 512);
            ptr += 512;
        }
        return bufsize;
    }

    int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
        (void) lun; (void) offset;

        // Belt and braces: refuse if write-protected or DIP-locked.
        if (gpio_get(PIN_WP) == 0 || !mode_has(MODE_WRITE_ENABLE))
            return -1;

        uint32_t block_count = bufsize / 512;

        if (mode_has(MODE_DEBUG_SERIAL)) {
            debug_serial_write("[I] WRITE lba=");
            debug_serial_write_dec32(lba);
            debug_serial_write(" n=");
            debug_serial_write_dec32(block_count);
            debug_serial_write("\r\n");
        }

        for (uint32_t i = 0; i < block_count; i++) {
            memcpy(shared_sector_buffer, buffer + i * 512, 512);
            shared_target_lba = lba + i;
            core1_result_ready = false;
            core1_write_request = true;

            __dmb();
            core1_request_pending = true;

            uint32_t wait_start = to_ms_since_boot(get_absolute_time());
            while (!core1_result_ready) {
                if (to_ms_since_boot(get_absolute_time()) - wait_start > 5000) {
                    if (mode_has(MODE_DEBUG_SERIAL))
                        debug_serial_write("[E] WRITE timeout\r\n");
                    core1_request_pending = false;
                    core1_write_request = false;
                    return -1;
                }
                tight_loop_contents();
            }

            __dmb();
            core1_write_request = false;

            if (!core1_result_success) {
                if (mode_has(MODE_DEBUG_SERIAL))
                    debug_serial_write("[W] WRITE fail\r\n");
                return -1;
            }
        }
        return bufsize;
    }

    int32_t tud_msc_scsi_cb (uint8_t lun, uint8_t const scsi_cmd[16], void* buffer, uint16_t bufsize) {
        void const* response = NULL;
        int32_t resplen = 0;

        switch (scsi_cmd[0]) {
            case 0x1A:
            {
                static uint8_t const mode_sense6_resp[] = { 0x03, 0x00, 0x00, 0x00 };
                response = mode_sense6_resp; resplen = sizeof(mode_sense6_resp); break;
            }
            case 0x5A:
            {
                static uint8_t const mode_sense10_resp[] = { 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
                response = mode_sense10_resp; resplen = sizeof(mode_sense10_resp); break;
            }
            case 0x23:
            {
                static uint8_t const format_capacities_resp[] = {
                    0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x0B, 0x40, 0x02, 0x00, 0x02, 0x00
                };
                response = format_capacities_resp; resplen = sizeof(format_capacities_resp); break;
            }
            case 0x1E:
                resplen = 0; break;
            case 0x04:
            {
                if (mode_has(MODE_DEBUG_SERIAL))
                    debug_serial_write("[I] SCSI FORMAT UNIT\r\n");
                scsi_format_requested = true;
                resplen = 0;
                break;
            }
            case 0x1B:
                resplen = 0; break;
            case 0x00:
                resplen = 0; break;
            default:
                if (mode_has(MODE_DEBUG_SERIAL)) {
                    debug_serial_write("[W] SCSI unk op=");
                    debug_serial_write_hex32(scsi_cmd[0]);
                    debug_serial_write("\r\n");
                }
                tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
                resplen = -1;
                break;
        }

        if (resplen > 0) {
            if ((uint32_t)resplen > bufsize) resplen = bufsize;
            if (response) memcpy(buffer, response, resplen);
        }
        return resplen;
    }

    bool tud_msc_is_writable_cb (uint8_t lun) {
        (void) lun;

        // WP pin takes priority: LOW means write-protected (tab open).
        if (gpio_get(PIN_WP) == 0) return false;

        // DIP switch acts as a secondary software lock.
        if (!mode_has(MODE_WRITE_ENABLE)) return false;

        return true;
    }
}
