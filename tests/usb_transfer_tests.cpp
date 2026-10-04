#include "pico/stdlib.h"
#include "tusb.h"
#include "shared_state.h"
#include "mode_config.h"
#include "debug_serial.h"
#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>

extern "C" int32_t tud_msc_read10_cb(uint8_t, uint32_t, uint32_t, void*, uint32_t);
extern "C" int32_t tud_msc_write10_cb(uint8_t, uint32_t, uint32_t, uint8_t*, uint32_t);
extern "C" int32_t tud_msc_scsi_cb(uint8_t, const uint8_t*, void*, uint16_t);
#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); std::exit(1); \
} } while (0)

volatile bool core1_request_pending, core1_write_request, core1_format_request;
volatile bool core1_result_ready, core1_result_success, core1_format_done, core1_format_success;
volatile bool format_in_progress, force_recalibrate, sense_media_changed, scsi_format_requested;
volatile bool disk_present = true;
volatile uint32_t shared_target_lba;
volatile uint8_t shared_sector_buffer[512], shared_format_cyl, shared_format_head;
volatile uint8_t shared_debug_state, shared_read_progress, shared_last_cyl, shared_last_head, shared_last_sec;
volatile uint32_t shared_debug_pll;
static uint64_t now;
static bool connected = true, debug = true, writable = true;
static uint32_t cdc_used, flushes, flush_budget = 10000;
static uint8_t sense, asc;
static std::array<char, 512> cdc_output;
uint64_t get_absolute_time() { return now; }
uint32_t to_ms_since_boot(uint64_t value) { return uint32_t(value / 1000); }
uint32_t time_us_32() { return uint32_t(now); }
void __dmb() {}
void sleep_us(uint64_t us) { now += us; }
void sleep_ms(uint32_t ms) { now += uint64_t(ms) * 1000; }
void tight_loop_contents() { CHECK(false); } // MSC callbacks must return, never spin.
void led_set_rgb(uint8_t, uint8_t, uint8_t) {}
bool gpio_get(unsigned) { return writable; }
bool mode_has_debug() { return debug; }
bool mode_has_write() { return writable; }
bool tud_msc_set_sense(uint8_t, uint8_t key, uint8_t code, uint8_t) {
    sense = key; asc = code; return true;
}
bool tud_cdc_connected() { return connected; }
uint32_t tud_cdc_write_available() { return 512 - cdc_used; }
uint32_t tud_cdc_write(const void* data, uint32_t length) {
    uint32_t accepted = length < 512 - cdc_used ? length : 512 - cdc_used;
    std::memcpy(cdc_output.data() + cdc_used, data, accepted);
    cdc_used += accepted;
    return accepted;
}
uint32_t tud_cdc_write_flush() {
    CHECK(++flushes <= flush_budget); // Flushing cannot free data without USB events.
    return 0;
}

static void complete(bool success, const uint8_t* read_data = nullptr) {
    CHECK(core1_request_pending);
    if (read_data) std::memcpy((void*)shared_sector_buffer, read_data, 512);
    core1_result_success = success;
    core1_write_request = false;
    core1_result_ready = true;
    core1_request_pending = false;
}

int main() {
    std::array<uint8_t, 512> data{}, output{};
    for (int i = 0; i < 512; ++i) data[i] = uint8_t(i);

    // Reproduce an attached serial terminal that stops reading: exactly the
    // old infinite flush loop. Calls must return even with zero CDC space.
    cdc_used = 512;
    flush_budget = 4;
    debug_serial_write("[I] READ lba=");
    debug_serial_write_dec32(149);
    debug_serial_write_hex32(0xffffffff);
    debug_serial_flush();
    CHECK(cdc_used == 512 && flushes == 4);
    flush_budget = 100000;

    // 150 KiB (300 sectors), plus 1 MiB of repeated transfers. Every request
    // yields while the drive is busy, logs only once, and retains one buffer.
    for (uint32_t lba = 0; lba < 2348; ++lba) {
        data[0] = uint8_t(lba);
        uint32_t before = flushes;
        CHECK(tud_msc_write10_cb(0, lba, 0, data.data(), 512) == 0);
        CHECK(core1_request_pending && core1_write_request && shared_target_lba == lba);
        for (int retry = 0; retry < 32; ++retry) {
            now += 1000;
            CHECK(tud_msc_write10_cb(0, lba, 0, data.data(), 512) == 0);
            CHECK(flushes == before + 3);
            CHECK(std::memcmp((const void*)shared_sector_buffer, data.data(), 512) == 0);
        }
        now += 200000;
        complete(true);
        CHECK(tud_msc_write10_cb(0, lba, 0, data.data(), 512) == 512);

        CHECK(tud_msc_read10_cb(0, lba, 0, output.data(), 512) == 0);
        CHECK(!core1_write_request && shared_target_lba == lba);
        for (int retry = 0; retry < 8; ++retry)
            CHECK(tud_msc_read10_cb(0, lba, 0, output.data(), 512) == 0);
        complete(true, data.data());
        CHECK(tud_msc_read10_cb(0, lba, 0, output.data(), 512) == 512);
        CHECK(data == output);
    }

    // Timeout retains core 1 ownership; a new command cannot overwrite it.
    CHECK(tud_msc_write10_cb(0, 12, 0, data.data(), 512) == 0);
    now += 5001000;
    CHECK(tud_msc_write10_cb(0, 12, 0, data.data(), 512) == -1);
    CHECK(sense == SCSI_SENSE_MEDIUM_ERROR && asc == 0x0c && core1_request_pending);
    std::array<uint8_t, 16> command{};
    command[0] = 0x35; // A reset must not make an outstanding write look flushed.
    CHECK(tud_msc_scsi_cb(0, command.data(), nullptr, 0) == -1);
    CHECK(sense == SCSI_SENSE_NOT_READY);
    CHECK(tud_msc_read10_cb(0, 13, 0, output.data(), 512) == -1);
    CHECK(shared_target_lba == 12);
    complete(true);
    CHECK(tud_msc_read10_cb(0, 13, 0, output.data(), 512) == 0);
    CHECK(shared_target_lba == 13);
    complete(true, data.data());
    CHECK(tud_msc_read10_cb(0, 13, 0, output.data(), 512) == 512);

    // A reset/retry with changed write data must not acknowledge old data.
    CHECK(tud_msc_write10_cb(0, 14, 0, data.data(), 512) == 0);
    data[1] ^= 1;
    CHECK(tud_msc_write10_cb(0, 14, 0, data.data(), 512) == 0);
    CHECK(((const uint8_t*)shared_sector_buffer)[1] != data[1]);
    complete(true);
    CHECK(tud_msc_write10_cb(0, 14, 0, data.data(), 512) == 0);
    CHECK(((const uint8_t*)shared_sector_buffer)[1] == data[1]);
    complete(false);
    CHECK(tud_msc_write10_cb(0, 14, 0, data.data(), 512) == -1);
    CHECK(sense == SCSI_SENSE_MEDIUM_ERROR && asc == 0x0c);

    CHECK(tud_msc_read10_cb(0, 15, 0, output.data(), 512) == 0);
    complete(false);
    CHECK(tud_msc_read10_cb(0, 15, 0, output.data(), 512) == -1);
    CHECK(sense == SCSI_SENSE_MEDIUM_ERROR && asc == 0x11);
    CHECK(tud_msc_scsi_cb(0, command.data(), nullptr, 0) == 0);

    writable = false;
    CHECK(tud_msc_write10_cb(0, 14, 0, data.data(), 512) == -1);
    CHECK(sense == SCSI_SENSE_DATA_PROTECT);
    writable = true;
    std::array<uint8_t, 1024> large{};
    CHECK(tud_msc_write10_cb(0, 20, 0, large.data(), 1024) == 0);
    complete(true);
    CHECK(tud_msc_write10_cb(0, 20, 0, large.data(), 1024) == 512);
    CHECK(tud_msc_write10_cb(0, 2880, 0, data.data(), 512) == -1);
    CHECK(tud_msc_write10_cb(0, 2879, 0, large.data(), 1024) == -1);
    CHECK(asc == 0x21);
    CHECK(tud_msc_read10_cb(0, 0, 1, output.data(), 512) == -1);
    disk_present = false;
    CHECK(tud_msc_read10_cb(0, 0, 0, output.data(), 512) == -1);
    CHECK(sense == SCSI_SENSE_NOT_READY && asc == 0x3a);
    disk_present = true;
    format_in_progress = true;
    CHECK(tud_msc_write10_cb(0, 20, 0, data.data(), 512) == -1);
    CHECK(sense == SCSI_SENSE_NOT_READY);
    format_in_progress = false;
    CHECK(!core1_request_pending);
    std::puts("Full debug FIFO, 150 KiB + 1 MiB transfers, retries, resets, timeouts and protection passed");
}
