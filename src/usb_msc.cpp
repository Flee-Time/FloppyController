#include "tusb.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "shared_state.h"
#include "mode_config.h"
#include "debug_serial.h"
#include "hw_config.h"
#include "usb_msc.h"
#include <string.h>

static bool new_request_allowed(uint8_t lun) {
    if (core1_request_pending || core1_format_request || format_in_progress || scsi_format_requested) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x07);
        return false;
    }
    if (!disk_present) {
        tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3A, 0x00);
        return false;
    }
    return true;
}

enum class IoKind { NONE, READ, WRITE };
struct PendingIo {
    IoKind kind = IoKind::NONE;
    uint32_t lba = 0;
    uint32_t started_ms = 0;
    bool timeout_reported = false;
    uint32_t last_report_ms = 0;
};
static PendingIo pending_io;

static const char* io_stage_name() {
    static const char* const names[] = {
        "idle", "spinup", "seek", "read", "encode", "dma-prepare",
        "prewrite", "locate-id", "emit", "dma-stop", "recovery",
        "verify", "done", "format-index"
    };
    unsigned stage = shared_io_stage;
    return stage < sizeof(names) / sizeof(names[0]) ? names[stage] : "unknown";
}

void usb_msc_poll() {
    if (pending_io.kind == IoKind::NONE || !mode_has_debug()) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - pending_io.last_report_ms < 1000) return;
    pending_io.last_report_ms = now;
    debug_serial_write(pending_io.kind == IoKind::WRITE ? "[W] WRITE waiting lba=" : "[W] READ waiting lba=");
    debug_serial_write_dec32(pending_io.lba);
    debug_serial_write(" stage=");
    debug_serial_write(io_stage_name());
    debug_serial_write(" ms=");
    debug_serial_write_dec32(now - pending_io.started_ms);
    __dmb();
    if (!core1_request_pending && core1_result_ready)
        debug_serial_write(" worker=finished usb=pending");
    debug_serial_write("\r\n");
}

static void log_io_failure(IoKind kind, uint32_t lba) {
    if (!mode_has_debug()) return;
    debug_serial_write(kind == IoKind::WRITE ? "[W] WRITE fail lba=" : "[W] READ fail lba=");
    debug_serial_write_dec32(lba);
    if (kind == IoKind::READ) {
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
    }
    debug_serial_write("\r\n");
}

static int32_t sector_transfer(uint8_t lun, uint32_t lba, uint32_t offset,
                               void* buffer, uint32_t bufsize, IoKind kind) {
    // The configured endpoint transfers full sectors. Larger buffers are
    // consumed one sector at a time, never copied wholesale into shared RAM.
    if (!buffer || offset || !bufsize || (bufsize % 512)) {
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x24, 0x00);
        return -1;
    }
    if (lba >= 2880 || bufsize / 512 > 2880 - lba) {
        tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x21, 0x00);
        return -1;
    }

    if (pending_io.kind != IoKind::NONE) {
        if (core1_request_pending || !core1_result_ready) {
            if (to_ms_since_boot(get_absolute_time()) - pending_io.started_ms > 5000) {
                if (!pending_io.timeout_reported && mode_has_debug())
                    debug_serial_write("[E] sector I/O timeout\r\n");
                pending_io.timeout_reported = true;
                tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR,
                                  kind == IoKind::WRITE ? 0x0C : 0x11, 0x00);
                return -1;
            }
            // TinyUSB retains the endpoint buffer and retries on return 0.
            // Return to its event loop so CDC, reset and control transfers
            // are serviced while core 1 performs the physical sector I/O.
            return 0;
        }
        __dmb();
        bool matches = pending_io.kind == kind && pending_io.lba == lba;
        if (matches && kind == IoKind::WRITE)
            matches = memcmp(buffer, (const void*)shared_sector_buffer, 512) == 0;
        bool success = core1_result_success;
        pending_io.kind = IoKind::NONE;
        if (matches) {
            if (!success) {
                log_io_failure(kind, lba);
                tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR,
                                  kind == IoKind::WRITE ? 0x0C : 0x11, 0x00);
                return -1;
            }
            if (kind == IoKind::READ)
                memcpy(buffer, (const void*)shared_sector_buffer, 512);
            else if (mode_has_debug()) {
                debug_serial_write("[I] WRITE verified lba=");
                debug_serial_write_dec32(lba);
                debug_serial_write("\r\n");
            }
            return 512;
        }
        // A USB reset may have abandoned the previous command. Discard its
        // result, then start the new request after core 1 releases the buffer.
    }

    if (!new_request_allowed(lun)) return -1;
    if (kind == IoKind::WRITE && (gpio_get(PIN_WP) == 0 || !mode_has_write())) {
        tud_msc_set_sense(lun, SCSI_SENSE_DATA_PROTECT, 0x27, 0x00);
        return -1;
    }
    if (mode_has_debug()) {
        debug_serial_write(kind == IoKind::WRITE ? "[I] WRITE lba=" : "[I] READ lba=");
        debug_serial_write_dec32(lba);
        debug_serial_write(" n=1\r\n");
    }
    if (kind == IoKind::WRITE)
        memcpy((void*)shared_sector_buffer, buffer, 512);
    shared_target_lba = lba;
    core1_write_request = kind == IoKind::WRITE;
    core1_result_ready = false;
    uint32_t started = to_ms_since_boot(get_absolute_time());
    pending_io = {kind, lba, started, false, started};
    __dmb();
    core1_request_pending = true;
    return 0;
}

extern "C" {
    bool tud_msc_test_unit_ready_cb(uint8_t lun) {
        (void) lun;

        if (core1_format_request || format_in_progress || scsi_format_requested) {
            if (mode_has_debug())
                debug_serial_write("[I] TUR: format in progress\r\n");
            tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x04);
            return false;
        }

        if (!disk_present) {
            if (mode_has_debug())
                debug_serial_write("[I] TUR: no disk\r\n");
            tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
            return false;
        }

        if (sense_media_changed) {
            if (mode_has_debug())
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

    int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                               void* buffer, uint32_t bufsize) {
        return sector_transfer(lun, lba, offset, buffer, bufsize, IoKind::READ);
    }

    int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                                uint8_t* buffer, uint32_t bufsize) {
        return sector_transfer(lun, lba, offset, buffer, bufsize, IoKind::WRITE);
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
                if (gpio_get(PIN_WP) == 0 || !mode_has_write()) {
                    tud_msc_set_sense(lun, SCSI_SENSE_DATA_PROTECT, 0x27, 0x00);
                    return -1;
                }
                if (core1_request_pending || core1_format_request || format_in_progress || scsi_format_requested) {
                    tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x04);
                    return -1;
                }
                if (mode_has_debug())
                    debug_serial_write("[I] SCSI FORMAT UNIT\r\n");
                scsi_format_requested = true;
                resplen = 0;
                break;
            }
            case 0x1B:
            case 0x35: // SYNCHRONIZE CACHE: writes commit before USB acknowledges them.
                if (core1_request_pending || core1_format_request || format_in_progress || scsi_format_requested) {
                    tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x04, 0x07);
                    return -1;
                }
                resplen = 0; break;
            case 0x00:
                resplen = 0; break;
            default:
                if (mode_has_debug()) {
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
        if (!mode_has_write()) return false;

        return true;
    }
}
