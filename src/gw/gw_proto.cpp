#include "tusb.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include <cstring>
#include <cstdlib>

#include "gw/gw_proto.h"
#include "gw/greasepack.h"
#include "hw_config.h"
#include "floppy_hw.h"
#include "led_drv.h"
#include "mode_config.h"
#include "usb_task.h"
#include "gw_writer.pio.h"

enum {
    CMD_GET_INFO       = 0,
    CMD_SEEK           = 2,
    CMD_HEAD           = 3,
    CMD_SET_PARAMS     = 4,
    CMD_GET_PARAMS     = 5,
    CMD_MOTOR          = 6,
    CMD_READ_FLUX      = 7,
    CMD_WRITE_FLUX     = 8,
    CMD_GET_FLUX_STATUS = 9,
    CMD_GET_INDEX_TIMES = 10,
    CMD_SELECT         = 12,
    CMD_DESELECT       = 13,
    CMD_SET_BUS_TYPE   = 14,
    CMD_SET_PIN        = 15,
    CMD_RESET          = 16,
    CMD_ERASE_FLUX     = 17,
    CMD_SOURCE_BYTES   = 18,
    CMD_SINK_BYTES     = 19,
    CMD_GET_PIN        = 20,
    CMD_TEST_MODE      = 21,
    CMD_NO_CLICK_STEP  = 22,
};

enum {
    ACK_OK            = 0,
    ACK_BAD_CMD       = 1,
    ACK_NO_INDEX      = 2,
    ACK_NO_TRK0       = 3,
    ACK_FLUX_OVERFLOW = 4,
    ACK_FLUX_UNDERFLOW= 5,
    ACK_WRPROT        = 6,
    ACK_NO_UNIT       = 7,
    ACK_NO_BUS        = 8,
    ACK_BAD_CYL       = 11,
    ACK_OUT_OF_SRAM   = 12,
};

enum {
    FLUXOP_INDEX = 1,
    FLUXOP_SPACE = 2,
};

#define GW_CMD_BUF_MAX   20
#define GW_FLUX_CAP_WORDS 105000
constexpr size_t GW_FLUX_CAP_BYTES = GW_FLUX_CAP_WORDS * sizeof(uint16_t);

static uint8_t  g_cmd_buf[GW_CMD_BUF_MAX];
static uint8_t  g_cmd_len;

static bool     gw_drive_selected;
static bool     gw_motor_on;
static uint8_t  gw_current_cyl;
static uint8_t  gw_current_head;

static PIO      gw_w_pio = pio1;
static uint     gw_w_sm;
static uint     gw_w_offset;
static bool     gw_writer_ready;

static uint8_t  gw_flux_status;
static uint8_t  gw_delays[16]; // 8 x uint16_t: select, step, seek_settle, motor, watchdog, pre_write, post_write, index_mask
static uint32_t bw_min_bytes, bw_min_us, bw_max_bytes, bw_max_us;

static void gw_init_writer(void) {
    if (gw_writer_ready) {
        pio_sm_set_enabled(gw_w_pio, gw_w_sm, false);
        pio_sm_unclaim(gw_w_pio, gw_w_sm);
        pio_remove_program(gw_w_pio, &gw_writer_program, gw_w_offset);
        gw_writer_ready = false;
    }
    gw_w_sm = pio_claim_unused_sm(gw_w_pio, false);
    if ((int)gw_w_sm < 0) return;
    gw_w_offset = pio_add_program(gw_w_pio, &gw_writer_program);
    gw_writer_program_init(gw_w_pio, gw_w_sm, gw_w_offset, PIN_WDATA);
    gw_writer_ready = true;
}

// --- DMA-based capture buffer ---
// Allocate once only when the boot DIP selects GreaseWeasel. MSC mode must
// not reserve 210 KB for an inactive protocol. Neither mode buffers files.
static uint16_t* cap_buf;
static uint32_t last_write_end_us;

static uint32_t gw_sample_freq(void) {
    return clock_get_hz(clk_sys) / 2;
}

static void gw_flush_tx(void) {
    tud_cdc_write_flush();
}

static void gw_send_response(const uint8_t* data, uint32_t len) {
    uint32_t w = 0;
    while (w < len) {
        usb_task_run();
        uint32_t avail = tud_cdc_write_available();
        if (avail == 0) {
            tud_cdc_write_flush();
            continue;
        }
        uint32_t n = len - w;
        if (n > avail) n = avail;
        tud_cdc_write((char*)(data + w), n);
        w += n;
    }
    gw_flush_tx();
}

static void gw_send_ack(uint8_t cmd, uint8_t code) {
    uint8_t buf[2] = { cmd, code };
    gw_send_response(buf, 2);
}

static bool gw_read_packet(void) {
    if (tud_cdc_available() < 2) return false;

    g_cmd_buf[0] = (uint8_t)tud_cdc_read_char();
    g_cmd_buf[1] = (uint8_t)tud_cdc_read_char();

    if (g_cmd_buf[1] < 2 || g_cmd_buf[1] > GW_CMD_BUF_MAX) {
        uint8_t bad = g_cmd_buf[0];
        for (int i = 2; i < g_cmd_buf[1]; i++) {
            while (!tud_cdc_available()) { usb_task_run(); tight_loop_contents(); }
            tud_cdc_read_char();
        }
        gw_send_ack(bad, ACK_BAD_CMD);
        return false;
    }

    g_cmd_len = g_cmd_buf[1];

    while (tud_cdc_available() < (int)(g_cmd_len - 2)) {
        usb_task_run();
        tight_loop_contents();
    }

    for (uint8_t i = 2; i < g_cmd_len; i++)
        g_cmd_buf[i] = (uint8_t)tud_cdc_read_char();

    return true;
}

static void cmd_get_info(void) {
    uint8_t subcmd = g_cmd_buf[2];

    uint8_t resp[34];
    resp[0] = CMD_GET_INFO;
    resp[1] = ACK_OK;
    memset(resp + 2, 0, 32);

    switch (subcmd) {
        case 0: {
            resp[2] = 1;
            resp[3] = 0;
            resp[4] = 1;
            resp[5] = 22;
            uint32_t sf = gw_sample_freq();
            memcpy(resp + 6, &sf, 4);
            resp[10] = 8;
            resp[11] = 0;
            resp[12] = 0;
            resp[13] = 0;
            uint16_t mhz = (uint16_t)(clock_get_hz(clk_sys) / 1000000);
            memcpy(resp + 14, &mhz, 2);
            uint16_t sram = 264;
            memcpy(resp + 16, &sram, 2);
            uint16_t usbbuf = 4;
            memcpy(resp + 18, &usbbuf, 2);
            break;
        }
        case 1: { // BandwidthStats
            memcpy(resp + 2, &bw_min_bytes, 4);
            memcpy(resp + 6, &bw_min_us, 4);
            memcpy(resp + 10, &bw_max_bytes, 4);
            memcpy(resp + 14, &bw_max_us, 4);
            break;
        }
        case 7: {
            uint32_t flags = 0;
            if (gw_motor_on) flags |= 2;
            uint32_t cyl = gw_current_cyl;
            memcpy(resp + 2, &flags, 4);
            memcpy(resp + 6, &cyl, 4);
            break;
        }
        default:
            resp[1] = ACK_BAD_CMD;
            break;
    }

    gw_send_response(resp, 34);
}

static void cmd_seek(void) {
    int16_t cyl;
    if (g_cmd_len == 3) {
        cyl = (int8_t)g_cmd_buf[2];
    } else if (g_cmd_len == 4) {
        cyl = (int16_t)((uint16_t)g_cmd_buf[2] | ((uint16_t)g_cmd_buf[3] << 8));
    } else {
        gw_send_ack(CMD_SEEK, ACK_BAD_CMD);
        return;
    }

    if (cyl < 0 || cyl > 83) {
        gw_send_ack(CMD_SEEK, ACK_BAD_CYL);
        return;
    }

    gw_drive_selected = true;
    drive_select(true);
    if (!gw_motor_on) { drive_motor(true); gw_motor_on = true; }
    sleep_ms(floppy_get_motor_delay_ms());

    {
        uint32_t post_w = floppy_get_post_write_us();
        if (post_w && last_write_end_us) {
            uint32_t dt = time_us_32() - last_write_end_us;
            if (dt < post_w) sleep_us(post_w - dt);
        }
    }

    seek_physical_track((uint8_t)cyl, gw_current_head);
    gw_current_cyl = (uint8_t)cyl;
    gw_send_ack(CMD_SEEK, ACK_OK);
}

static void cmd_head(void) {
    uint8_t head = g_cmd_buf[2];
    if (head > 1) {
        gw_send_ack(CMD_HEAD, ACK_BAD_CMD);
        return;
    }
    gw_current_head = head;
    seek_physical_track(gw_current_cyl, head);
    gw_send_ack(CMD_HEAD, ACK_OK);
}

static void cmd_motor(void) {
    uint8_t unit = g_cmd_buf[2];
    uint8_t on   = g_cmd_buf[3];
    (void)unit;

    gw_motor_on = on != 0;
    drive_motor(gw_motor_on);
    if (!gw_motor_on) { drive_select(false); gw_drive_selected = false; }
    gw_send_ack(CMD_MOTOR, ACK_OK);
}

static void cmd_select(void) {
    uint8_t unit = g_cmd_buf[2];
    (void)unit;
    gw_drive_selected = true;
    drive_select(true);
    gw_send_ack(CMD_SELECT, ACK_OK);
}

static void cmd_deselect(void) {
    gw_drive_selected = false;
    drive_select(false);
    gw_send_ack(CMD_DESELECT, ACK_OK);
}

static void cmd_set_bus_type(void) {
    (void)g_cmd_buf[2];
    gw_send_ack(CMD_SET_BUS_TYPE, ACK_OK);
}

static void cmd_get_pin(void) {
    uint8_t pin = g_cmd_buf[2];
    bool level = false;
    switch (pin) {
        case 26: level = gpio_get(PIN_TRACK0); break;
        case 28: level = gpio_get(PIN_WP); break;
        case 2:  level = gpio_get(PIN_DSKCHG); break;
        case 0:  level = gpio_get(PIN_INDEX); break;
    }
    uint8_t resp[3] = { CMD_GET_PIN, ACK_OK, (uint8_t)(level ? 1 : 0) };
    gw_send_response(resp, 3);
}

static void cmd_set_pin(void) {
    uint8_t pin = g_cmd_buf[2];
    uint8_t lvl = g_cmd_buf[3];
    switch (pin) {
        case 2:  set_density(lvl != 0); break;
        case 24: gpio_put(PIN_MOTEA, !lvl); break;
        case 23: drive_select(lvl != 0); break;
        case 12: gpio_put(PIN_WGATE, !lvl); break;
        case 4:  gpio_put(PIN_SIDE1, lvl); break;
        case 5:  gpio_put(PIN_DIR, lvl); break;
        case 6:  gpio_put(PIN_STEP, lvl); break;
    }
    gw_send_ack(CMD_SET_PIN, ACK_OK);
}

static void cmd_reset(void) {
    gw_drive_selected = false;
    gw_motor_on = false;
    drive_select(false);
    drive_motor(false);
    last_write_end_us = 0;
    gw_flux_status = ACK_OK;
    gw_send_ack(CMD_RESET, ACK_OK);
}


static volatile bool idx_fell;
static volatile uint32_t idx_deadline;

static void idx_irq_cb(uint gpio, uint32_t events) {
    if (gpio == PIN_INDEX && (events & GPIO_IRQ_EDGE_FALL))
        idx_fell = true;
}


static void cmd_read_flux(void) {
    if (!cap_buf) {
        gw_flux_status = ACK_OUT_OF_SRAM;
        gw_send_ack(CMD_READ_FLUX, ACK_OUT_OF_SRAM);
        return;
    }
    uint32_t ticks_limit;
    uint16_t revs;
    memcpy(&ticks_limit, g_cmd_buf + 2, 4);
    memcpy(&revs, g_cmd_buf + 6, 2);

    gw_send_ack(CMD_READ_FLUX, ACK_OK);

    if (!gw_drive_selected) { drive_select(true); gw_drive_selected = true; }
    if (!gw_motor_on) { drive_motor(true); gw_motor_on = true; sleep_ms(floppy_get_motor_delay_ms()); }

    // ticks_limit-based (no index sync) for FluxEngine
    if (revs == 0) {
        if (ticks_limit == 0) ticks_limit = 1000000;
        pio_sm_clear_fifos(flux_pio, flux_sm);
        uint32_t td = 0; uint32_t cc = 0;
        uint32_t deadline = time_us_32() + (ticks_limit / 100) + 5000000;
        while (td < ticks_limit && cc < GW_FLUX_CAP_WORDS && time_us_32() < deadline) {
            while (!pio_sm_is_rx_fifo_empty(flux_pio, flux_sm) && cc < GW_FLUX_CAP_WORDS) {
                uint32_t v = pio_sm_get(flux_pio, flux_sm);
                uint32_t tk = 0xFFFFFFFF - v;
                if (tk <= 65500) { td += tk; cap_buf[cc++] = (uint16_t)tk; if (td >= ticks_limit) break; }
            }
        }
        uint8_t ob[512]; uint32_t op = 0;
        for (uint32_t i = 0; i < cc; i++) {
            uint8_t* ep = greasepack(ob + op, ob + sizeof(ob), cap_buf[i]);
            size_t n = (size_t)(ep - (ob + op)); if (n > 0) op += (uint32_t)n;
            if (op >= 400) { gw_send_response(ob, op); op = 0; }
        }
        if (op > 0) gw_send_response(ob, op);
        { uint8_t t = 0; gw_send_response(&t, 1); }
        while (tud_cdc_available()) tud_cdc_read_char();
        return;
    }

    // RPM mode (revs=2, ticks_limit=0): uses time_us_32() polling for
    // deterministic rotational period measurement, avoiding the variable
    // latency of PIO FIFO drain during revs-based IRQ index detection.
    if (ticks_limit == 0 && revs == 2) {
        uint32_t t_sync;
        {
            int to = 0;
            while (true) {
                while (gpio_get(PIN_INDEX) == 1) {
                    if (++to > 12000000) { while (tud_cdc_available()) tud_cdc_read_char(); return; }
                }
                uint32_t t_low = time_us_32();
                while (gpio_get(PIN_INDEX) == 0) {
                    if (time_us_32() - t_low > 4000) { while (tud_cdc_available()) tud_cdc_read_char(); return; }
                }
                if (time_us_32() - t_low >= 200) break;
                to = 0;
            }
            t_sync = time_us_32();
        }

        uint32_t idx_mask = floppy_get_index_mask_us();
        uint32_t dl = t_sync + (idx_mask ? idx_mask : 150000);
        while (true) {
            if ((int32_t)(time_us_32() - dl) >= 0 && gpio_get(PIN_INDEX) == 0) {
                uint32_t t_low = time_us_32();
                while (gpio_get(PIN_INDEX) == 0) {
                    if (time_us_32() - t_low > 4000) break;
                }
                if (time_us_32() - t_low >= 200) break;
            }
            tight_loop_contents();
        }
        uint32_t t_end = time_us_32();
        uint32_t period_us = t_end - t_sync;
        if (period_us < 50000 || period_us > 500000) period_us = 200000;
        uint32_t ticks_total = (uint32_t)((uint64_t)period_us * gw_sample_freq() / 1000000);
        uint32_t cc = 0;
        while (ticks_total > 0 && cc < GW_FLUX_CAP_WORDS) {
            uint16_t v = ticks_total > 65500 ? 65500 : (uint16_t)ticks_total;
            cap_buf[cc++] = v;
            ticks_total -= v;
        }
        { uint8_t idx[6] = { 0xFF, FLUXOP_INDEX, 1, 1, 1, 1 }; gw_send_response(idx, 6); }
        uint8_t ob[512]; uint32_t op = 0;
        for (uint32_t i = 0; i < cc; i++) {
            uint8_t* ep = greasepack(ob + op, ob + sizeof(ob), cap_buf[i]);
            size_t n = (size_t)(ep - (ob + op)); if (n > 0) op += (uint32_t)n;
            if (op >= 400) { gw_send_response(ob, op); op = 0; }
            if (i + 1 == cc) {
                if (op > 0) { gw_send_response(ob, op); op = 0; }
                uint8_t idx[6] = { 0xFF, FLUXOP_INDEX, 1, 1, 1, 1 }; gw_send_response(idx, 6);
            }
        }
        if (op > 0) gw_send_response(ob, op);
        { uint8_t t = 0; gw_send_response(&t, 1); }
        while (tud_cdc_available()) tud_cdc_read_char();
        return;
    }

    // General revs-based capture with hardware IRQ index detection
    idx_fell = false;
    gpio_set_irq_enabled_with_callback(PIN_INDEX, GPIO_IRQ_EDGE_FALL, true, &idx_irq_cb);
    {
        int to = 0;
        while (!idx_fell) { if (++to > 12000000) break; tight_loop_contents(); }
        idx_fell = false;
    }
    gpio_set_irq_enabled(PIN_INDEX, GPIO_IRQ_EDGE_FALL, false);

    pio_sm_clear_fifos(flux_pio, flux_sm);

    idx_fell = false;
    gpio_set_irq_enabled_with_callback(PIN_INDEX, GPIO_IRQ_EDGE_FALL, true, &idx_irq_cb);
    idx_deadline = time_us_32() + 100000;

    uint32_t cc = 0; uint32_t ip[8]; uint16_t ic = 0; ip[ic++] = 0;
    uint32_t rdeadline = time_us_32() + 5000000;

    while (ic < revs && cc < GW_FLUX_CAP_WORDS && time_us_32() < rdeadline) {
        while (!pio_sm_is_rx_fifo_empty(flux_pio, flux_sm) && cc < GW_FLUX_CAP_WORDS) {
            uint32_t v = pio_sm_get(flux_pio, flux_sm);
            uint32_t tk = 0xFFFFFFFF - v;
            if (tk <= 65500) cap_buf[cc++] = (uint16_t)tk;
        }
        if (idx_fell) {
            if ((int32_t)(time_us_32() - idx_deadline) >= 0 && gpio_get(PIN_INDEX) == 0) {
                if (ic < 8) ip[ic++] = cc;
                idx_deadline = time_us_32() + 150000;
            }
            idx_fell = false;
        }
    }
    gpio_set_irq_enabled(PIN_INDEX, GPIO_IRQ_EDGE_FALL, false);

    if (ic < revs && ic < 8 && cc > 0) ip[ic++] = cc;

    // TRANSMIT
    { uint8_t idx[6] = { 0xFF, FLUXOP_INDEX, 1, 1, 1, 1 }; gw_send_response(idx, 6); }
    uint8_t ob[512]; uint32_t op = 0; uint16_t ii = 1;
    for (uint32_t i = 0; i < cc; i++) {
        uint32_t val = cap_buf[i];
        uint8_t* ep = greasepack(ob + op, ob + sizeof(ob), val);
        size_t n = (size_t)(ep - (ob + op)); if (n > 0) op += (uint32_t)n;
        if (op >= 400) { gw_send_response(ob, op); op = 0; }
        if (ii < ic && i + 1 == ip[ii]) {
            if (op > 0) { gw_send_response(ob, op); op = 0; }
            uint8_t idx[6] = { 0xFF, FLUXOP_INDEX, 1, 1, 1, 1 }; gw_send_response(idx, 6); ii++;
        }
    }
    if (op > 0) gw_send_response(ob, op);
    { uint8_t t = 0; gw_send_response(&t, 1); }
    while (tud_cdc_available()) tud_cdc_read_char();
}

static void cmd_write_flux(void) {
    if (!cap_buf) {
        gw_flux_status = ACK_OUT_OF_SRAM;
        gw_send_ack(CMD_WRITE_FLUX, ACK_OUT_OF_SRAM);
        return;
    }
    bool cue_at_index = g_cmd_buf[2] != 0;
    bool terminate_at_index = (g_cmd_len >= 4) ? (g_cmd_buf[3] != 0) : false;

    if (!mode_has_write() || gpio_get(PIN_WP) == 0) {
        gw_send_ack(CMD_WRITE_FLUX, ACK_WRPROT);
        return;
    }

    gw_send_ack(CMD_WRITE_FLUX, ACK_OK);

    if (!gw_drive_selected) {
        drive_select(true); gw_drive_selected = true;
    }
    if (!gw_motor_on) {
        drive_motor(true); gw_motor_on = true;
        sleep_ms(floppy_get_motor_delay_ms());
    }

    set_density(true); // Assert HD density for write

    gw_init_writer();
    if (!gw_writer_ready) {
        uint8_t c;
        do {
            while (!tud_cdc_available()) { usb_task_run(); }
            c = (uint8_t)tud_cdc_read_char();
        } while (c != 0);
        gw_send_ack(CMD_GET_FLUX_STATUS, ACK_OK);
        return;
    }

    {
        uint32_t pre_w = floppy_get_pre_write_us();
        if (pre_w) sleep_us(pre_w);
    }

    pio_sm_set_enabled(gw_w_pio, gw_w_sm, false);
    pio_sm_clear_fifos(gw_w_pio, gw_w_sm);
    pio_sm_restart(gw_w_pio, gw_w_sm);

    // Read all raw bytes first
    uint8_t* raw = (uint8_t*)cap_buf;
    uint32_t raw_len = 0;
    while (true) {
        while (!tud_cdc_available()) { usb_task_run(); }
        uint8_t b = (uint8_t)tud_cdc_read_char();
        if (b == 0) break;
        if (raw_len < GW_FLUX_CAP_BYTES)
            raw[raw_len++] = b;
    }

    // Re-cue on index right before writing (Adafruit style)
    if (cue_at_index) {
        int timeout = 0;
        while (gpio_get(PIN_INDEX) == 1) {
            if (++timeout > 20000) break;
            sleep_us(50);
        }
        while (gpio_get(PIN_INDEX) == 0) {
            if (++timeout > 20000) break;
            sleep_us(50);
        }
    }

    // PIO init
    pio_sm_set_enabled(gw_w_pio, gw_w_sm, false);
    pio_sm_clear_fifos(gw_w_pio, gw_w_sm);
    pio_sm_restart(gw_w_pio, gw_w_sm);

    // Single short pre-load to avoid garbage OSR on first OUT
    pio_sm_put(gw_w_pio, gw_w_sm, 200);

    uint8_t* gp = raw;
    uint8_t* end = raw + raw_len;

    write_gate(true);
    sleep_us(200);
    pio_sm_set_enabled(gw_w_pio, gw_w_sm, true);

    while (gp < end) {
        unsigned val = greaseunpack(&gp, end, true);
        if (val == 0 || val == 0xFFFF) break;
        unsigned dv = val / 2;
        dv = (dv > 34) ? (dv - 34) : 0;
        if (dv < 2) dv = 2;
        if (dv > 0xFFFF) dv = 0xFFFF;
        pio_sm_put_blocking(gw_w_pio, gw_w_sm, (uint16_t)dv);
    }

    while (!pio_sm_is_tx_fifo_empty(gw_w_pio, gw_w_sm))
        tight_loop_contents();
    pio_sm_set_enabled(gw_w_pio, gw_w_sm, false);
    write_gate(false);
    gpio_put(PIN_WDATA, 1); // restore WDATA inactive HIGH

    // Fully release gw_writer SM to avoid PIO stall on subsequent reads
    pio_sm_unclaim(gw_w_pio, gw_w_sm);
    pio_remove_program(gw_w_pio, &gw_writer_program, gw_w_offset);
    gw_writer_ready = false;

    // Minimal write-to-read recovery. The configurable post_write_us
    // guard in cmd_seek() enforces the full requirement before a
    // seek-into-read.
    sleep_ms(20);

    last_write_end_us = time_us_32();
    usb_task_run();
    { uint8_t sync = 0; gw_send_response(&sync, 1); }
}

static void cmd_get_flux_status(void) {
    uint8_t st = gw_flux_status;
    gw_flux_status = ACK_OK;
    gw_send_ack(CMD_GET_FLUX_STATUS, st);
}

static void cmd_get_params(void) {
    uint8_t idx = g_cmd_buf[2];
    uint8_t nr  = g_cmd_buf[3];

    switch (idx) {
        case 0: { // Delays
            if (nr > sizeof(gw_delays)) nr = sizeof(gw_delays);
            uint8_t resp[2 + sizeof(gw_delays)];
            resp[0] = CMD_GET_PARAMS;
            resp[1] = ACK_OK;
            memcpy(resp + 2, gw_delays, nr);
            gw_send_response(resp, 2 + nr);
            break;
        }
        default:
            gw_send_ack(CMD_GET_PARAMS, ACK_BAD_CMD);
            break;
    }
}

static void cmd_set_params(void) {
    uint8_t idx = g_cmd_buf[2];
    uint8_t datalen = g_cmd_len - 3;

    switch (idx) {
        case 0: { // Delays
            if (datalen > sizeof(gw_delays)) datalen = sizeof(gw_delays);
            memcpy(gw_delays, g_cmd_buf + 3, datalen);
            if (datalen >= 2) {
                uint16_t sel_us;
                memcpy(&sel_us, gw_delays + 0, 2);
                floppy_set_select_delay_us(sel_us);
            }
            if (datalen >= 4) {
                uint16_t step_us;
                memcpy(&step_us, gw_delays + 2, 2);
                floppy_set_step_delay_ms(step_us / 1000);
            }
            if (datalen >= 6) {
                uint16_t settle_ms;
                memcpy(&settle_ms, gw_delays + 4, 2);
                floppy_set_settle_time_ms(settle_ms);
            }
            if (datalen >= 8) {
                uint16_t motor_ms;
                memcpy(&motor_ms, gw_delays + 6, 2);
                floppy_set_motor_delay_ms(motor_ms);
            }
            if (datalen >= 10) {
                uint16_t wd_ms;
                memcpy(&wd_ms, gw_delays + 8, 2);
                floppy_set_watchdog_ms(wd_ms);
            }
            if (datalen >= 12) {
                uint16_t pre_w;
                memcpy(&pre_w, gw_delays + 10, 2);
                floppy_set_pre_write_us(pre_w);
            }
            if (datalen >= 14) {
                uint16_t post_w;
                memcpy(&post_w, gw_delays + 12, 2);
                floppy_set_post_write_us(post_w);
            }
            if (datalen >= 16) {
                uint16_t im;
                memcpy(&im, gw_delays + 14, 2);
                floppy_set_index_mask_us(im);
            }
            gw_send_ack(CMD_SET_PARAMS, ACK_OK);
            break;
        }
        default:
            gw_send_ack(CMD_SET_PARAMS, ACK_BAD_CMD);
            break;
    }
}

static void cmd_get_index_times(void) {
    uint16_t revs;
    memcpy(&revs, g_cmd_buf + 2, 2);
    if (revs == 0 || revs > 10) revs = 1;

    if (!gw_drive_selected) { drive_select(true); gw_drive_selected = true; }
    if (!gw_motor_on) { drive_motor(true); gw_motor_on = true; sleep_ms(floppy_get_motor_delay_ms()); }

    uint32_t times[11];
    uint8_t tc = 0;

    {
        int to = 0;
        while (true) {
            while (gpio_get(PIN_INDEX) == 1) {
                if (++to > 12000000) { gw_send_ack(CMD_GET_INDEX_TIMES, ACK_NO_INDEX); return; }
            }
            uint32_t t_low = time_us_32();
            while (gpio_get(PIN_INDEX) == 0) {
                if (time_us_32() - t_low > 4000) { gw_send_ack(CMD_GET_INDEX_TIMES, ACK_NO_INDEX); return; }
            }
            if (time_us_32() - t_low >= 200) break;
            to = 0;
        }
    }

    uint32_t t0 = time_us_32();
    times[tc++] = 0;
    uint32_t dl = time_us_32() + 150000;

    while (tc <= revs) {
        if ((int32_t)(time_us_32() - dl) >= 0 && gpio_get(PIN_INDEX) == 0) {
            times[tc++] = (time_us_32() - t0) * (gw_sample_freq() / 1000000);
            dl = time_us_32() + 150000;
        }
        tight_loop_contents();
    }

    uint8_t resp[2 + 11 * 4];
    resp[0] = CMD_GET_INDEX_TIMES;
    resp[1] = ACK_OK;
    memcpy(resp + 2, times, (revs + 1) * 4);
    gw_send_response(resp, 2 + (revs + 1) * 4);
}

static void cmd_erase_flux(void) {
    uint32_t ticks;
    memcpy(&ticks, g_cmd_buf + 2, 4);

    if (!mode_has_write() || gpio_get(PIN_WP) == 0) {
        gw_send_ack(CMD_ERASE_FLUX, ACK_WRPROT);
        return;
    }

    if (!gw_drive_selected) {
        drive_select(true); gw_drive_selected = true;
    }
    if (!gw_motor_on) {
        drive_motor(true); gw_motor_on = true;
        sleep_ms(floppy_get_motor_delay_ms());
    }

    write_gate(true);
    uint32_t start = time_us_32();
    while (time_us_32() - start < ticks / 50) {
        usb_task_run();
    }
    write_gate(false);
    sleep_ms(5); // write-to-read recovery time
    last_write_end_us = time_us_32();
    uint8_t sync = 0;
    gw_send_response(&sync, 1);
}

static void cmd_source_bytes(void) {
    uint32_t nr, seed;
    memcpy(&nr, g_cmd_buf + 2, 4);
    memcpy(&seed, g_cmd_buf + 6, 4);

    gw_send_ack(CMD_SOURCE_BYTES, ACK_OK);

    uint32_t t0 = time_us_32();
    uint32_t nr_total = nr;
    uint32_t randnum = seed;
    uint8_t buf[512];
    while (nr > 0) {
        uint32_t chunk = nr > sizeof(buf) ? sizeof(buf) : nr;
        for (uint32_t i = 0; i < chunk; i++) {
            buf[i] = (uint8_t)randnum;
            if (randnum & 0x01)
                randnum = (randnum >> 1) ^ 0x80000062;
            else
                randnum >>= 1;
        }
        gw_send_response(buf, chunk);
        nr -= chunk;
    }

    uint32_t elapsed = time_us_32() - t0;
    if (elapsed == 0) elapsed = 1;
    bw_min_bytes = bw_max_bytes = nr_total;
    bw_min_us = bw_max_us = elapsed;
}

static void cmd_sink_bytes(void) {
    uint32_t nr, seed;
    memcpy(&nr, g_cmd_buf + 2, 4);
    memcpy(&seed, g_cmd_buf + 6, 4);
    (void)seed;

    gw_send_ack(CMD_SINK_BYTES, ACK_OK);

    uint32_t t0 = time_us_32();
    uint32_t nr_total = nr;
    uint8_t buf[256];
    while (nr > 0) {
        while (!tud_cdc_available()) { usb_task_run(); }
        uint32_t n = tud_cdc_available();
        if (n > sizeof(buf)) n = sizeof(buf);
        if (n > nr) n = nr;
        tud_cdc_read(buf, n);
        nr -= n;
    }

    uint32_t elapsed = time_us_32() - t0;
    if (elapsed == 0) elapsed = 1;
    bw_min_bytes = bw_max_bytes = nr_total;
    bw_min_us = bw_max_us = elapsed;

    uint8_t ack = ACK_OK;
    gw_send_response(&ack, 1);
}

static void cmd_no_click_step(void) {
    gpio_put(PIN_DIR, 1);
    sleep_us(1);
    gpio_put(PIN_STEP, 0);
    sleep_us(10);
    gpio_put(PIN_STEP, 1);
    sleep_us(1);
    gpio_put(PIN_DIR, 0);
    gw_send_ack(CMD_NO_CLICK_STEP, ACK_OK);
}

static void cmd_test_mode(void) {
    uint8_t sub = g_cmd_buf[2];
    switch (sub) {
        case 0:
            gw_send_ack(CMD_TEST_MODE, ACK_OK);
            break;
        case 1: {
            uint8_t resp[GW_CMD_BUF_MAX];
            resp[0] = CMD_TEST_MODE;
            resp[1] = ACK_OK;
            uint8_t n = g_cmd_len - 3;
            if (n > GW_CMD_BUF_MAX - 2) n = GW_CMD_BUF_MAX - 2;
            memcpy(resp + 2, g_cmd_buf + 3, n);
            gw_send_response(resp, 2 + n);
            break;
        }
        default:
            gw_send_ack(CMD_TEST_MODE, ACK_OK);
            break;
    }
}

static void gw_dispatch(void) {
    uint8_t cmd = g_cmd_buf[0];

    switch (cmd) {
        case CMD_GET_INFO:       cmd_get_info(); break;
        case CMD_SEEK:           cmd_seek(); break;
        case CMD_HEAD:           cmd_head(); break;
        case CMD_MOTOR:          cmd_motor(); break;
        case CMD_SELECT:         cmd_select(); break;
        case CMD_DESELECT:       cmd_deselect(); break;
        case CMD_SET_BUS_TYPE:   cmd_set_bus_type(); break;
        case CMD_GET_PIN:        cmd_get_pin(); break;
        case CMD_SET_PIN:        cmd_set_pin(); break;
        case CMD_RESET:          cmd_reset(); break;
        case CMD_READ_FLUX:      cmd_read_flux(); break;
        case CMD_WRITE_FLUX:     cmd_write_flux(); break;
        case CMD_GET_FLUX_STATUS: cmd_get_flux_status(); break;
        case CMD_GET_PARAMS:     cmd_get_params(); break;
        case CMD_SET_PARAMS:     cmd_set_params(); break;
        case CMD_GET_INDEX_TIMES: cmd_get_index_times(); break;
        case CMD_ERASE_FLUX:     cmd_erase_flux(); break;
        case CMD_SOURCE_BYTES:   cmd_source_bytes(); break;
        case CMD_SINK_BYTES:     cmd_sink_bytes(); break;
        case CMD_TEST_MODE:      cmd_test_mode(); break;
        case CMD_NO_CLICK_STEP:  cmd_no_click_step(); break;
        default:
            gw_send_ack(cmd, ACK_BAD_CMD);
            break;
    }
}

void gw_proto_init(void) {
    if (!cap_buf && mode_has_gw())
        cap_buf = static_cast<uint16_t*>(malloc(GW_FLUX_CAP_BYTES));
    led_set_rgb(0, 0, 0);
    gw_drive_selected = false;
    gw_motor_on = false;
    gw_current_cyl = 0;
    gw_current_head = 0;
    gw_flux_status = ACK_OK;
    {
        uint16_t defaults[] = { 0, 3000, 15, 600, 10000, 2000, 2000, 5000 };
        memcpy(gw_delays, defaults, sizeof(gw_delays));
        floppy_set_step_delay_ms(3000 / 1000);
        floppy_set_settle_time_ms(15);
        floppy_set_motor_delay_ms(600);
        floppy_set_watchdog_ms(10000);
        floppy_set_pre_write_us(2000);
        floppy_set_post_write_us(2000);
        floppy_set_index_mask_us(5000);
    }
}

void gw_proto_poll(void) {
    // Auto motor-off after 10s idle (check even when no CDC data)
    static uint32_t last_activity;
    uint32_t now = time_us_32();
    if (tud_cdc_available()) {
        last_activity = now;
    } else if (gw_motor_on && now - last_activity > floppy_get_watchdog_ms() * 1000) {
        drive_motor(false);
        drive_select(false);
        gw_drive_selected = false;
        gw_motor_on = false;
    }

    if (!tud_cdc_available()) return;
    if (!gw_read_packet()) return;
    gw_dispatch();
}
