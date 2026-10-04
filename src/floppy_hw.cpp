#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "floppy_hw.h"
#include "hw_config.h"
#include "shared_state.h"
#include "flux_reader.pio.h"
#include "flux_writer.pio.h"

volatile int current_track = -1;
static uint8_t current_head = 0xFF;

static uint32_t g_select_delay_us = 0;
static uint32_t g_step_delay_ms = 3;
static uint32_t g_settle_time_ms = 15;
static uint32_t g_motor_delay_ms = 600;
static uint32_t g_watchdog_ms = 10000;
static uint32_t g_pre_write_us = 2000;
static uint32_t g_post_write_us = 2000;
static uint32_t g_index_mask_us = 5000;

void floppy_set_select_delay_us(uint32_t us) { g_select_delay_us = us; }
void floppy_set_step_delay_ms(uint32_t ms) { g_step_delay_ms = ms; }
void floppy_set_settle_time_ms(uint32_t ms) { g_settle_time_ms = ms; }
void floppy_set_motor_delay_ms(uint32_t ms) { g_motor_delay_ms = ms; }
void floppy_set_watchdog_ms(uint32_t ms) { g_watchdog_ms = ms; }
uint32_t floppy_get_motor_delay_ms(void) { return g_motor_delay_ms; }
uint32_t floppy_get_watchdog_ms(void) { return g_watchdog_ms; }

void floppy_set_pre_write_us(uint32_t us) { g_pre_write_us = us; }
void floppy_set_post_write_us(uint32_t us) { g_post_write_us = us; }
void floppy_set_index_mask_us(uint32_t us) { g_index_mask_us = us; }
uint32_t floppy_get_pre_write_us(void) { return g_pre_write_us; }
uint32_t floppy_get_post_write_us(void) { return g_post_write_us; }
uint32_t floppy_get_index_mask_us(void) { return g_index_mask_us; }

PIO flux_pio = pio0;
volatile uint flux_sm;
volatile uint write_sm;
volatile uint write_offset;

void init_hardware() {
    gpio_init(PIN_DRVSB); gpio_set_dir(PIN_DRVSB, GPIO_OUT); gpio_put(PIN_DRVSB, 1);
    gpio_init(PIN_MOTEA); gpio_set_dir(PIN_MOTEA, GPIO_OUT); gpio_put(PIN_MOTEA, 1);

    gpio_init(PIN_DIR); gpio_set_dir(PIN_DIR, GPIO_OUT); gpio_put(PIN_DIR, 1);
    gpio_init(PIN_STEP); gpio_set_dir(PIN_STEP, GPIO_OUT); gpio_put(PIN_STEP, 1);
    gpio_init(PIN_SIDE1); gpio_set_dir(PIN_SIDE1, GPIO_OUT); gpio_put(PIN_SIDE1, 1);

    gpio_init(PIN_WDATA); gpio_set_dir(PIN_WDATA, GPIO_OUT); gpio_put(PIN_WDATA, 1);
    gpio_init(PIN_WGATE); gpio_set_dir(PIN_WGATE, GPIO_OUT); gpio_put(PIN_WGATE, 1);

    gpio_init(PIN_DENSITY); gpio_set_dir(PIN_DENSITY, GPIO_OUT); gpio_put(PIN_DENSITY, 0); // DD default

    gpio_init(PIN_INDEX);  gpio_set_dir(PIN_INDEX,  GPIO_IN); gpio_pull_up(PIN_INDEX);
    gpio_init(PIN_TRACK0); gpio_set_dir(PIN_TRACK0, GPIO_IN); gpio_pull_up(PIN_TRACK0);
    gpio_init(PIN_WP);     gpio_set_dir(PIN_WP,     GPIO_IN); gpio_pull_up(PIN_WP);
    gpio_init(PIN_RDATA);  gpio_set_dir(PIN_RDATA,  GPIO_IN); gpio_pull_up(PIN_RDATA);
    gpio_init(PIN_DSKCHG); gpio_set_dir(PIN_DSKCHG, GPIO_IN); gpio_pull_up(PIN_DSKCHG);

    flux_sm = pio_claim_unused_sm(flux_pio, true);
    uint flux_offset = pio_add_program(flux_pio, &flux_reader_program);
    flux_reader_program_init(flux_pio, flux_sm, flux_offset, PIN_RDATA);

    write_sm = pio_claim_unused_sm(pio1, true);
    write_offset = pio_add_program(pio1, &flux_writer_program);
    flux_writer_program_init(pio1, write_sm, write_offset, PIN_WDATA);
}

void drive_select(bool active) {
    gpio_put(PIN_DRVSB, !active);
    if (active && g_select_delay_us) busy_wait_us_32(g_select_delay_us);
}

void drive_motor(bool active) {
    gpio_put(PIN_MOTEA, !active);
}

void drive_step(bool direction_in) {
    gpio_put(PIN_DIR, direction_in ? 0 : 1);
    gpio_put(PIN_STEP, 0);
    busy_wait_ms(g_step_delay_ms);
    gpio_put(PIN_STEP, 1);
    busy_wait_ms(g_step_delay_ms);
}

void seek_track_0() {
    // The caller must have the motor already spinning. Step outward a few
    // tracks to guarantee we clear the track 0 sensor, then step inward
    // until the sensor triggers.
    for (int i = 0; i < 5; i++) drive_step(true);
    busy_wait_ms(50);

    int timeout = 100;
    while (gpio_get(PIN_TRACK0) == 1 && timeout > 0) {
        drive_step(false);
        timeout--;
    }
    current_track = 0;

    // Let the head and mechanics settle before attempting reads.
    busy_wait_ms(20);
}


void seek_physical_track(uint8_t target_track, uint8_t target_head) {
    if (force_recalibrate) {
        seek_track_0();
        force_recalibrate = false;
    }
    if (current_track == -1) seek_track_0();

    bool head_changed = (target_head != current_head);
    gpio_put(PIN_SIDE1, (target_head == 0) ? 1 : 0);
    current_head = target_head;

    uint32_t settle = g_settle_time_ms;

    if (target_track == current_track) {
        if (head_changed) busy_wait_ms(settle);
        return;
    }

    bool direction = (target_track > current_track);

    int steps = abs(target_track - current_track);

    for (int i = 0; i < steps; i++) {
        drive_step(direction);
    }

    current_track = target_track;
    busy_wait_ms(settle);
}

#define DISK_PROBE_INTERVAL_MS 2000

void poll_disk_change(uint32_t now) {
    static uint32_t last_probe = 0;
    static bool initial_probe_done = false;

    drive_select(true);
    busy_wait_us_32(100);

    bool latch_set = (gpio_get(PIN_DSKCHG) == 0);

    if (!latch_set) {
        // Latch is clear: we can trust whatever state we currently believe.
        // Mark our initial probe as done since the latch state tells us
        // definitively that the current belief is correct.
        if (!initial_probe_done) {
            // On the very first poll after boot with the latch already
            // clear, we need to probe to discover whether a disk is
            // actually loaded. The latch being clear could mean either
            // "no disk was ever inserted" or "disk was inserted and the
            // latch was already cleared by a step" — we can't tell.
            // Fall through to the probe logic below so we step the head
            // and resolve the ambiguity.
        } else {
            drive_select(false);
            return;
        }
    } else if (disk_present) {
        disk_present = false;
        sense_media_changed = true;
        current_track = -1;
        drive_select(false);
        return;
    }

    // Throttle probing except on the very first poll, where we must
    // determine whether a disk was already inserted before power-on.
    if (initial_probe_done && now - last_probe < DISK_PROBE_INTERVAL_MS) {
        drive_select(false);
        return;
    }
    last_probe = now;
    initial_probe_done = true;

    drive_motor(true);
    drive_step(true);
    drive_step(false);
    busy_wait_ms(5);

    bool still_set = (gpio_get(PIN_DSKCHG) == 0);
    if (!still_set) {
        disk_present = true;
        sense_media_changed = true;
        seek_track_0();
    }

    drive_motor(false);
    drive_select(false);
}

void write_gate(bool active) {
    gpio_put(PIN_WGATE, !active);
}

void set_density(bool high) {
    gpio_put(PIN_DENSITY, high ? 1 : 0);
}
