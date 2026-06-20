#pragma once
#include <stdint.h>
#include <stdbool.h>

// --- Core 0 to Core 1 Requests ---
extern volatile bool core1_request_pending;
extern volatile uint32_t shared_target_lba;

// --- Core 1 to Core 0 Responses ---
extern volatile bool core1_result_ready;
extern volatile bool core1_result_success;
extern uint8_t shared_sector_buffer[512];

// --- Hardware State (Updated by interrupts/Core 1, read by Core 0) ---
extern volatile bool sense_media_changed;
extern volatile bool disk_present;