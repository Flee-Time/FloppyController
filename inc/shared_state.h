#pragma once
#include <stdint.h>
#include <stdbool.h>

// --- Core 0 to Core 1 Requests ---
extern volatile bool core1_request_pending;
extern volatile bool core1_write_request;
extern volatile uint32_t shared_target_lba;

// --- Core 1 to Core 0 Responses ---
extern volatile bool core1_result_ready;
extern volatile bool core1_result_success;
extern uint8_t shared_sector_buffer[512];

// --- Core 1 Debug Info ---
// 0=idle, 1=no sync mark found, 2=wrong sector/ID CRC fail,
// 3=data CRC fail, 4=timeout, 5=bandpass reject
extern volatile uint8_t shared_debug_state;
extern volatile uint32_t shared_debug_pll;
extern volatile uint32_t shared_ticks_sample[8];
extern volatile uint8_t shared_ticks_count;
extern volatile uint8_t shared_read_progress;
extern volatile uint8_t shared_last_cyl;
extern volatile uint8_t shared_last_head;
extern volatile uint8_t shared_last_sec;

// --- Hardware State (Updated by Core 1, read by Core 0) ---
extern volatile bool sense_media_changed;
extern volatile bool disk_present;