#pragma once
#include <stdint.h>
enum {
    SCSI_SENSE_NOT_READY = 2, SCSI_SENSE_MEDIUM_ERROR = 3,
    SCSI_SENSE_ILLEGAL_REQUEST = 5, SCSI_SENSE_UNIT_ATTENTION = 6,
    SCSI_SENSE_DATA_PROTECT = 7
};
struct cdc_line_coding_t {};
bool tud_msc_set_sense(uint8_t, uint8_t, uint8_t, uint8_t);
bool tud_cdc_connected();
uint32_t tud_cdc_write(const void*, uint32_t);
uint32_t tud_cdc_write_available();
uint32_t tud_cdc_write_flush();
inline uint32_t tud_cdc_write_char(char c) { return tud_cdc_write(&c, 1); }
