#include "pico/stdlib.h"
#include "mfm_writer.h"
#include "floppy_hw.h"

// TODO: MSC write path needs ground-up rewrite.
// The flux_writer PIO + MFM encoder produce a repeating 0x9249 pattern
// on disk regardless of input data. Suspect the half-cell-to-WDATA
// mapping in the PIO program or the packing order in the encoder.
// Until fixed, individual sector writes will fail and format is
// disabled.

bool write_physical_sector(uint8_t target_cyl, uint8_t target_head,
                           uint8_t target_sec, const uint8_t* data) {
    (void)target_cyl; (void)target_head; (void)target_sec; (void)data;
    return false;
}

bool format_physical_track(uint8_t cyl, uint8_t head) {
    (void)cyl; (void)head;
    return false;
}
