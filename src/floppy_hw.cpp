#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "floppy_hw.h"
#include "hw_config.h"
#include "led_drv.h"
#include "shared_state.h"
#include "flux_reader.pio.h"

int current_track = -1;

PIO flux_pio = pio0;
uint flux_sm;

void init_hardware() {
    gpio_init(PIN_DRVSB); gpio_set_dir(PIN_DRVSB, GPIO_OUT); gpio_put(PIN_DRVSB, 1);
    gpio_init(PIN_MOTEA); gpio_set_dir(PIN_MOTEA, GPIO_OUT); gpio_put(PIN_MOTEA, 1);
    
    gpio_init(PIN_DIR); gpio_set_dir(PIN_DIR, GPIO_OUT); gpio_put(PIN_DIR, 1);
    gpio_init(PIN_STEP); gpio_set_dir(PIN_STEP, GPIO_OUT); gpio_put(PIN_STEP, 1);
    gpio_init(PIN_SIDE1); gpio_set_dir(PIN_SIDE1, GPIO_OUT); gpio_put(PIN_SIDE1, 1);
    
    gpio_init(PIN_TRACK0); gpio_set_dir(PIN_TRACK0, GPIO_IN); gpio_pull_up(PIN_TRACK0);
    gpio_init(PIN_DSKCHG); gpio_set_dir(PIN_DSKCHG, GPIO_IN); gpio_pull_up(PIN_DSKCHG);
    gpio_init(PIN_RDATA);  gpio_set_dir(PIN_RDATA, GPIO_IN);  gpio_pull_up(PIN_RDATA);

    flux_sm = pio_claim_unused_sm(flux_pio, true);
    uint flux_offset = pio_add_program(flux_pio, &flux_reader_program);
    flux_reader_program_init(flux_pio, flux_sm, flux_offset, PIN_RDATA);
}

void drive_select(bool active) {
    gpio_put(PIN_DRVSB, !active); 
}

void drive_motor(bool active) {
    gpio_put(PIN_MOTEA, !active); 
}

void drive_step(bool direction_in) {
    gpio_put(PIN_DIR, direction_in ? 0 : 1);
    gpio_put(PIN_STEP, 0);
    sleep_ms(10);
    gpio_put(PIN_STEP, 1);
    sleep_ms(10); 
}

void seek_track_0() {
    // Note: We now assume the main worker loop has already turned the motor ON
    
    for(int i = 0; i < 5; i++) drive_step(true); 
    sleep_ms(50); 

    int timeout = 100; 
    while (gpio_get(PIN_TRACK0) == 1 && timeout > 0) {
        drive_step(false);
        timeout--;
    }
    current_track = 0; 
}


void seek_physical_track(uint8_t target_track, uint8_t target_head) {
    if (current_track == -1) seek_track_0();
    
    gpio_put(PIN_SIDE1, (target_head == 0) ? 1 : 0);

    if (target_track == current_track) return; 

    bool direction = (target_track > current_track); 
    
    int steps = abs(target_track - current_track);

    for (int i = 0; i < steps; i++) {
        drive_step(direction);
    }
    
    current_track = target_track;
    sleep_ms(15); 
}

// How often (ms) we're willing to physically step the head just to test
// whether new media has shown up, while we currently believe the drive is
// empty. Real floppy drives only clear DSKCHG via a STEP pulse issued with
// media present -- a freshly-inserted disk does NOT clear the latch by
// itself. We still want to stay quiet/idle most of the time, so we only
// probe this often instead of on every poll.
#define DISK_PROBE_INTERVAL_MS 2000

void poll_disk_change(uint32_t now) {
    static uint32_t last_probe = 0;

    drive_select(true);
    sleep_us(100); // let the open-collector line settle

    bool latch_set = (gpio_get(PIN_DSKCHG) == 0); // active-low: asserted == change latched

    if (!latch_set) {
        // Latch clear -- whatever we currently believe (present or absent)
        // is correct and confirmed. Nothing to do.
        drive_select(false);
        return;
    }

    if (disk_present) {
        // We thought a disk was in there, and the latch just set itself.
        // That happens automatically and immediately on removal -- no step
        // is needed to know this one, unlike insertion.
        disk_present = false;
        sense_media_changed = true;
        current_track = -1;
        led_set_rgb(255, 165, 0); // amber: removed
        drive_select(false);
        return;
    }

    // We believe the drive is empty. The latch being set doesn't tell us
    // anything new by itself -- it's just leftover state from the last
    // removal until we actually test it. Periodically nudge the head to
    // check whether new media has shown up.
    if (now - last_probe < DISK_PROBE_INTERVAL_MS) {
        drive_select(false);
        return;
    }
    last_probe = now;

    drive_motor(true);
    drive_step(true);
    drive_step(false); // back to where we started -- net position unchanged
    sleep_ms(5);

    bool still_set = (gpio_get(PIN_DSKCHG) == 0);
    if (!still_set) {
        // The step reset the latch -- there's really a disk under the
        // heads now. Calibrate against it.
        disk_present = true;
        sense_media_changed = true;
        seek_track_0();
        led_set_rgb(128, 0, 128); // magenta: inserted
    }

    drive_motor(false);
    drive_select(false);
}
