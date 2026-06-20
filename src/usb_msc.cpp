#include "tusb.h"
#include "pico/stdlib.h"
#include "shared_state.h"
#include <string.h>

extern "C" {
    bool tud_msc_test_unit_ready_cb(uint8_t lun) {
        (void) lun;
        
        if (!disk_present) {
            tud_msc_set_sense(lun, SCSI_SENSE_NOT_READY, 0x3a, 0x00);
            sense_media_changed = true;
            return false;
        }

        if (sense_media_changed) {
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

        for (uint32_t i = 0; i < block_count; i++) {
            shared_target_lba = lba + i;
            core1_result_ready = false;
            core1_request_pending = true;

            // Core1 does the physical read completely independently of the
            // USB stack, so all we need to do is wait for it. IMPORTANT:
            // don't call tud_task() in here -- we were called FROM
            // tud_task(), and re-entering it mid-transfer is an unsupported
            // pattern that can corrupt the stack's internal state. This was
            // likely a contributor to the "request not supported" mount
            // failures.
            uint32_t wait_start = to_ms_since_boot(get_absolute_time());
            while (!core1_result_ready) {
                if (to_ms_since_boot(get_absolute_time()) - wait_start > 1000) {
                    // Core1 didn't answer in a reasonable time -- fail the
                    // command instead of hanging the USB stack forever.
                    tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
                    core1_request_pending = false;
                    return -1;
                }
                tight_loop_contents();
            }

            if (!core1_result_success) {
                tud_msc_set_sense(lun, SCSI_SENSE_MEDIUM_ERROR, 0x11, 0x00);
                return -1; 
            }
            
            memcpy(ptr, shared_sector_buffer, 512);
            ptr += 512; 
        }
        return bufsize; 
    }

    int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
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
            case 0x1B: 
                resplen = 0; break;
            default:
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
        return false;
    }
}
