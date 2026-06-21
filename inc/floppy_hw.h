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
void set_density(bool high);

void floppy_set_select_delay_us(uint32_t us);
void floppy_set_step_delay_ms(uint32_t ms);
void floppy_set_settle_time_ms(uint32_t ms);
void floppy_set_motor_delay_ms(uint32_t ms);
void floppy_set_watchdog_ms(uint32_t ms);
uint32_t floppy_get_motor_delay_ms(void);
uint32_t floppy_get_watchdog_ms(void);

void floppy_set_pre_write_us(uint32_t us);
void floppy_set_post_write_us(uint32_t us);
void floppy_set_index_mask_us(uint32_t us);
uint32_t floppy_get_pre_write_us(void);
uint32_t floppy_get_post_write_us(void);
uint32_t floppy_get_index_mask_us(void);
