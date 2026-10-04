#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/sync.h"
#include "mfm_writer.h"
#include "mfm_encoder.h"
#include "mfm_decoder.h"
#include "floppy_hw.h"
#include "hw_config.h"
#include "mode_config.h"
#include "flux_writer.pio.h"
#include <cstring>
#include <cstdlib>
#include <memory>

namespace {
bool writable() {
    return mode_has_write() && gpio_get(PIN_WP) != 0;
}

// Finite PIO program: DMA completion only means the FIFO was filled.
// The relative PIO IRQ proves every half-cell, including the CRC, finished.
class FluxWrite {
    int dma = -1;
    int words = 0;
public:
    bool prepare(const uint32_t* stream, int count) {
        if (!writable() || !stream || count <= 0) return false;
        dma = dma_claim_unused_channel(false);
        if (dma < 0) return false;
        words = count;
        write_gate(false);
        flux_writer_program_init(pio1, write_sm, write_offset, PIN_WDATA);
        pio_interrupt_clear(pio1, write_sm);
        pio1->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + write_sm);
        pio_sm_put(pio1, write_sm, uint32_t(count) * 32 - 1);

        dma_channel_config config = dma_channel_get_default_config(dma);
        channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
        channel_config_set_read_increment(&config, true);
        channel_config_set_write_increment(&config, false);
        channel_config_set_dreq(&config, pio_get_dreq(pio1, write_sm, true));
        channel_config_set_high_priority(&config, true);
        dma_channel_configure(dma, &config, &pio1->txf[write_sm], stream, count, true);

        uint32_t start = time_us_32();
        while (!pio_sm_is_tx_fifo_full(pio1, write_sm) && dma_channel_is_busy(dma)) {
            if (time_us_32() - start > 1000) return false;
            tight_loop_contents();
        }
        return true;
    }

    bool run(bool stop_at_index = false, uint32_t id_end_us = 0,
             uint32_t start_delay_us = 0) {
        if (dma < 0 || !writable()) return false;
        uint32_t irq_state = save_and_disable_interrupts();
        if (start_delay_us) {
            if (time_us_32() - id_end_us > start_delay_us) {
                restore_interrupts(irq_state);
                return false;
            }
            while (time_us_32() - id_end_us < start_delay_us) tight_loop_contents();
        }
        pio_sm_set_enabled(flux_pio, flux_sm, false);
        uint32_t start = time_us_32();
        bool index_high = false;
        bool ok = false;
        write_gate(true);
        pio_sm_set_enabled(pio1, write_sm, true);
        while (time_us_32() - start < uint32_t(words) * 32 + 1000) {
            // Any stall is an underrun, since completion uses an explicit IRQ.
            if (pio1->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + write_sm))) break;
            if (pio_interrupt_get(pio1, write_sm)) {
                ok = true;
                break;
            }
            if (stop_at_index) {
                if (gpio_get(PIN_INDEX)) index_high = true;
                else if (index_high) break; // Never wrap over sector 1.
            }
            if (!gpio_get(PIN_WP)) break;
        }
        write_gate(false);
        pio_sm_set_enabled(pio1, write_sm, false);
        restore_interrupts(irq_state);
        dma_channel_abort(dma);
        pio_sm_clear_fifos(pio1, write_sm);
        pio_sm_set_pins_with_mask(pio1, write_sm, 1u << PIN_WDATA, 1u << PIN_WDATA);
        sleep_us(floppy_get_post_write_us());
        pio_sm_clear_fifos(flux_pio, flux_sm);
        pio_sm_set_enabled(flux_pio, flux_sm, true);
        return ok;
    }

    ~FluxWrite() {
        if (dma >= 0) {
            write_gate(false);
            pio_sm_set_enabled(pio1, write_sm, false);
            dma_channel_abort(dma);
            dma_channel_unclaim(dma);
        }
    }
};

bool next_index(uint32_t* timestamp) {
    uint32_t start = time_us_32();
    while (!gpio_get(PIN_INDEX))
        if (time_us_32() - start > 500000) return false;
    while (gpio_get(PIN_INDEX))
        if (time_us_32() - start > 500000) return false;
    *timestamp = time_us_32();
    return true;
}
} // namespace

bool write_physical_sector(uint8_t target_cyl, uint8_t target_head,
                           uint8_t target_sec, const uint8_t* data) {
    if (!data || target_cyl >= 80 || target_head >= 2 ||
        target_sec < 1 || target_sec > 18 || !writable()) return false;

    static uint32_t stream[MFM_SECTOR_WORDS];
    int words = mfm_encode_sector(data, stream);
    FluxWrite writer;
    if (!writer.prepare(stream, words)) return false;
    sleep_us(floppy_get_pre_write_us()); // Before alignment, never inside the field.

    uint32_t id_end;
    if (!locate_physical_sector(target_cyl, target_head, target_sec, &id_end))
        return false; // Invalid/missing ID: WGATE has never been asserted.

    // Standard IBM HD GAP2 is 22 bytes at 16 us/byte. Start a new sync/DAM
    // here; writing only data after an already-decoded DAM starts too late.
    // Decoding adds a few us of latency, absorbed by the 12-byte sync and GAP3.
    constexpr uint32_t gap2_us = 22 * 16;
    if (!writer.run(false, id_end, gap2_us)) return false;

    static uint8_t verify[512];
    return read_physical_sector(target_cyl, target_head, target_sec, verify) &&
           memcmp(verify, data, sizeof(verify)) == 0;
}

bool format_physical_track(uint8_t cyl, uint8_t head) {
    if (cyl >= 80 || head >= 2 || !writable()) return false;
    uint32_t first, second;
    if (!next_index(&first) || !next_index(&second)) return false;
    uint32_t period = second - first;
    // Leave a 1 ms guard before the next index. Reject an unsupported spindle
    // speed instead of truncating a data field or overwriting sector 1.
    if (period < uint32_t(MFM_TRACK_MIN_WORDS) * 32 + 1000 ||
        period > uint32_t(MFM_TRACK_MAX_WORDS) * 32 + 1000) return false;
    int words = (period - 1000) / 32;
    // FORMAT alone needs a track-sized buffer. Release it after this track;
    // ordinary sector copies use only their fixed 1,064-byte stream.
    std::unique_ptr<uint32_t, decltype(&free)> stream(
        static_cast<uint32_t*>(malloc(size_t(words) * sizeof(uint32_t))), &free);
    if (!stream || format_track_encode(cyl, head, stream.get(), words) != words) return false;
    FluxWrite writer;
    if (!writer.prepare(stream.get(), words)) return false;
    if (!next_index(&first) || !writer.run(true)) return false;

    static uint8_t verify[512];
    for (uint8_t sec = 1; sec <= 18; ++sec) {
        if (!read_physical_sector(cyl, head, sec, verify)) return false;
        for (uint8_t byte : verify) if (byte != 0xE5) return false;
    }
    return true;
}
