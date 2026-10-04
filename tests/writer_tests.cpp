#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "floppy_hw.h"
#include "hw_config.h"
#include "mfm_writer.h"
#include "mfm_encoder.h"
#include "mode_config.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); std::exit(1); \
} } while (0)

static TestPio reader, writer;
PIO flux_pio = &reader, pio1 = &writer;
volatile uint flux_sm = 0, write_sm = 1, write_offset = 0;
static uint32_t now, id_end, started, stopped, length, count_header;
static int gates;
static bool claimed, writing, gate, reader_enabled, irq_enabled, write_enabled, wp;
static bool fail_claim, fail_prepare, fail_id, fail_irq, fail_stall, fail_read, corrupt_read, drop_wp;
static uint32_t id_latency;
static const uint32_t* stream;

static void reset() {
    now = id_end = started = stopped = length = count_header = gates = 0;
    claimed = writing = gate = false;
    reader_enabled = irq_enabled = write_enabled = wp = true;
    fail_claim = fail_prepare = fail_id = fail_irq = fail_stall = false;
    fail_read = corrupt_read = drop_wp = false;
    id_latency = 0;
}
uint32_t time_us_32() { return ++now; }
void sleep_us(uint64_t us) { CHECK(irq_enabled); now += uint32_t(us); }
void tight_loop_contents() {}
bool gpio_get(unsigned pin) {
    if (pin == PIN_WP) return wp && !(drop_wp && writing && now - started >= 100);
    if (pin == PIN_INDEX) return now % 200000 >= 2000;
    return true;
}
bool mode_has_write() { return write_enabled; }
void write_gate(bool on) {
    if (on) { CHECK(!gate); ++gates; started = now; }
    else if (gate) stopped = now;
    gate = on;
}
uint32_t floppy_get_pre_write_us() { return 2000; }
uint32_t floppy_get_post_write_us() { return 2000; }
uint32_t save_and_disable_interrupts() { bool old = irq_enabled; irq_enabled = false; return old; }
void restore_interrupts(uint32_t old) { irq_enabled = old != 0; }
void flux_writer_program_init(PIO, uint, uint, uint) { writing = false; }
void pio_interrupt_clear(PIO, uint) {}
bool pio_interrupt_get(PIO, uint) { return !fail_irq && now - started >= length * 32 + 1; }
void pio_sm_put(PIO, uint, uint32_t word) { count_header = word; }
uint pio_get_dreq(PIO, uint, bool tx) { CHECK(tx); return 9; }
bool pio_sm_is_tx_fifo_full(PIO, uint) { return !fail_prepare; }
void pio_sm_set_enabled(PIO pio, uint, bool enabled) {
    if (pio == flux_pio) reader_enabled = enabled;
    else { if (!enabled && writing) CHECK(!gate); writing = enabled; }
}
void pio_sm_clear_fifos(PIO, uint) {}
void pio_sm_set_pins_with_mask(PIO, uint, uint32_t value, uint32_t mask) {
    CHECK((value & mask) == (1u << PIN_WDATA));
}
uint32_t test_stall_flags() {
    return fail_stall && writing && now - started >= 100 ? 1u << (24 + write_sm) : 0;
}
int dma_claim_unused_channel(bool) { if (fail_claim) return -1; CHECK(!claimed); claimed = true; return 3; }
dma_channel_config dma_channel_get_default_config(unsigned) { return {}; }
void channel_config_set_transfer_data_size(dma_channel_config*, unsigned size) { CHECK(size == DMA_SIZE_32); }
void channel_config_set_read_increment(dma_channel_config*, bool on) { CHECK(on); }
void channel_config_set_write_increment(dma_channel_config*, bool on) { CHECK(!on); }
void channel_config_set_dreq(dma_channel_config*, unsigned req) { CHECK(req == 9); }
void channel_config_set_high_priority(dma_channel_config*, bool on) { CHECK(on); }
void dma_channel_configure(unsigned, const dma_channel_config*, volatile void* to,
                           const void* from, unsigned words, bool start) {
    CHECK(to == &pio1->txf[write_sm] && start);
    stream = static_cast<const uint32_t*>(from);
    length = words;
    CHECK(count_header == words * 32 - 1);
}
bool dma_channel_is_busy(unsigned) { return true; }
void dma_channel_abort(unsigned) { CHECK(!gate); }
void dma_channel_unclaim(unsigned) { CHECK(claimed); claimed = false; }
bool locate_physical_sector(uint8_t cyl, uint8_t head, uint8_t sec, uint32_t* boundary) {
    CHECK(cyl == 3 && head == 1 && sec == 7);
    CHECK(!gate && claimed && length == MFM_SECTOR_WORDS);
    *boundary = id_end = now - id_latency;
    return !fail_id;
}
bool read_physical_sector(uint8_t, uint8_t, uint8_t, uint8_t* data) {
    CHECK(!gate && !writing && reader_enabled && irq_enabled);
    if (fail_read) return false;
    for (int i = 0; i < 512; ++i) {
        int index = i + 16;
        uint16_t raw = uint16_t(stream[index / 2] >> (index % 2 ? 0 : 16));
        uint8_t value = 0;
        for (int b = 7; b >= 0; --b) value = uint8_t((value << 1) | ((raw >> (b * 2)) & 1));
        data[i] = value;
    }
    if (corrupt_read) data[100] ^= 1;
    return true;
}

static uint8_t data[512];
static bool write() { return write_physical_sector(3, 1, 7, data); }
static void cleanup() { CHECK(!gate && !writing && !claimed && reader_enabled && irq_enabled); }
int main() {
    for (int i = 0; i < 512; ++i) data[i] = uint8_t(i);
    reset(); CHECK(write()); cleanup();
    CHECK(gates == 1 && started - id_end >= 352 && started - id_end <= 355);
    CHECK(stopped - started >= 8512 && stopped - started <= 8516);

    reset(); wp = false; CHECK(!write()); cleanup(); CHECK(gates == 0);
    reset(); write_enabled = false; CHECK(!write()); cleanup(); CHECK(gates == 0);
    reset(); fail_claim = true; CHECK(!write()); cleanup(); CHECK(gates == 0);
    reset(); fail_prepare = true; CHECK(!write()); cleanup(); CHECK(gates == 0);
    reset(); fail_id = true; CHECK(!write()); cleanup(); CHECK(gates == 0);
    reset(); id_latency = 353; CHECK(!write()); cleanup(); CHECK(gates == 0);
    reset(); fail_stall = true; CHECK(!write()); cleanup(); CHECK(stopped - started < 150);
    reset(); drop_wp = true; CHECK(!write()); cleanup(); CHECK(stopped - started < 150);
    reset(); fail_irq = true; CHECK(!write()); cleanup(); CHECK(stopped - started < 9515);
    reset(); fail_read = true; CHECK(!write()); cleanup();
    reset(); corrupt_read = true; CHECK(!write()); cleanup();
    reset(); CHECK(!write_physical_sector(80, 1, 7, data)); cleanup(); CHECK(gates == 0);
    std::puts("Sector alignment, DMA setup, completion, protection, failure cleanup and verification passed");
}
