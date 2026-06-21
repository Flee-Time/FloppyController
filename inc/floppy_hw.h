#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h"

extern volatile int current_track;

extern PIO flux_pio;
extern volatile uint flux_sm;
extern volatile uint write_sm;

void init_hardware();
void drive_select(bool active);
void drive_motor(bool active);
void seek_track_0();
void seek_physical_track(uint8_t target_track, uint8_t target_head);
void poll_disk_change(uint32_t now);
void write_gate(bool active);
