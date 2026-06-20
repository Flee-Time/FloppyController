#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "hardware/pio.h" 

extern int current_track;

extern PIO flux_pio;
extern uint flux_sm;

void init_hardware();
void drive_select(bool active);
void drive_motor(bool active);
void seek_track_0();
void seek_physical_track(uint8_t target_track, uint8_t target_head);

// Polls the (latched) DSKCHG line and updates disk_present / current_track /
// sense_media_changed as needed. `now` is the caller's current
// to_ms_since_boot() timestamp, used to throttle how often we physically
// step the head while probing for newly-inserted media. Safe to call
// frequently (e.g. every poll loop iteration); it only touches the hardware
// when there's actually something to check.
void poll_disk_change(uint32_t now);
