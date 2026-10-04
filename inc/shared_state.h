#pragma once
#include <stdint.h>
#include <stdbool.h>

// Cross-core handshake protocol:
//   Producer: write payload → __dmb() → set ready flag
//   Consumer: wait for ready flag → __dmb() → read payload

// --- Core 0 to Core 1 Requests (Writer: Core0, Reader: Core1) ---
extern volatile bool core1_request_pending;
extern volatile bool core1_write_request;
extern volatile bool core1_format_request;
extern volatile uint32_t shared_target_lba;

// --- Core 1 to Core 0 Responses (Writer: Core1, Reader: Core0) ---
extern volatile bool core1_result_ready;
extern volatile bool core1_result_success;
extern volatile bool core1_format_done;
extern volatile bool core1_format_success;
extern volatile uint8_t shared_sector_buffer[512];

// --- Format state (Core 0 drives iteration) ---
extern volatile bool format_in_progress;
extern volatile uint8_t shared_format_cyl;
extern volatile uint8_t shared_format_head;

// Force core1 to recalibrate track 0 (e.g. after full format).
// Core0 sets, core1 checks at top of seek_physical_track then clears.
extern volatile bool force_recalibrate;

// --- Core 1 Debug Info ---
// 0=idle, 1=no sync mark found, 2=wrong sector/ID CRC fail,
// 3=data CRC fail, 4=timeout, 5=bandpass reject
extern volatile uint8_t shared_debug_state;
extern volatile uint32_t shared_debug_pll;
extern volatile uint8_t shared_read_progress;
extern volatile uint8_t shared_last_cyl;
extern volatile uint8_t shared_last_head;
extern volatile uint8_t shared_last_sec;

// Core 1 records stages without making USB calls in timing-sensitive code.
enum IoStage : uint8_t {
    IO_IDLE, IO_SPINUP, IO_SEEK, IO_READ, IO_ENCODE, IO_DMA_PREPARE,
    IO_PREWRITE, IO_LOCATE_ID, IO_EMIT, IO_DMA_STOP, IO_RECOVERY,
    IO_VERIFY, IO_DONE, IO_FORMAT_INDEX
};
extern volatile IoStage shared_io_stage;

// --- Hardware State (Updated by Core 1, read by Core 0) ---
extern volatile bool sense_media_changed;
extern volatile bool disk_present;

// --- Format trigger from SCSI callback ---
extern volatile bool scsi_format_requested;
