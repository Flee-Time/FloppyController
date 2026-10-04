#include "mfm_encoder.h"
#include "mfm_decoder.h"
#include "floppy_hw.h"
#include <array>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <vector>

PIO flux_pio = nullptr;
volatile uint flux_sm = 0;
volatile uint8_t shared_debug_state, shared_read_progress;
volatile uint8_t shared_last_cyl, shared_last_head, shared_last_sec;
volatile uint32_t shared_debug_pll;
static std::vector<uint32_t> flux;
static size_t cursor;
static uint64_t now_us;
void pio_sm_clear_fifos(PIO, uint) { cursor = 0; }
bool pio_sm_is_rx_fifo_empty(PIO, uint) { return cursor == flux.size(); }
uint32_t pio_sm_get(PIO, uint) {
    uint32_t ticks = flux[cursor++];
    now_us += ticks / 100;
    return 0xffffffffu - ticks;
}
uint64_t get_absolute_time() { return ++now_us; }
uint32_t to_ms_since_boot(uint64_t t) { return uint32_t(t / 1000); }
uint32_t time_us_32() { return uint32_t(now_us); }

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); std::exit(1); \
} } while (0)

static uint16_t raw_byte(const uint32_t* words, int byte) {
    return uint16_t(words[byte / 2] >> (byte % 2 ? 0 : 16));
}

// Independent polynomial long division, rather than the encoder's CRC loop.
static uint16_t crc(const std::vector<uint8_t>& bytes) {
    uint32_t remainder = 0xffff;
    for (uint8_t b : bytes) {
        remainder ^= uint32_t(b) << 8;
        for (int i = 0; i < 8; ++i) {
            remainder <<= 1;
            if (remainder & 0x10000) remainder ^= 0x11021;
        }
    }
    return uint16_t(remainder);
}

static uint8_t decode(uint16_t raw, int& prev) {
    uint8_t value = 0;
    for (int i = 7; i >= 0; --i) {
        int d = (raw >> (i * 2)) & 1;
        int clock = (raw >> (i * 2 + 1)) & 1;
        CHECK(clock == (!d && !prev));
        value = uint8_t((value << 1) | d);
        prev = d;
    }
    return value;
}

static uint16_t encode_byte(uint8_t byte, int& prev) {
    uint16_t raw = 0;
    for (int i = 7; i >= 0; --i) {
        int data = (byte >> i) & 1;
        raw = uint16_t((raw << 2) | ((!prev && !data) << 1) | data);
        prev = data;
    }
    return raw;
}

static void replace_byte(uint32_t* words, int byte, uint16_t raw) {
    int shift = byte % 2 ? 0 : 16;
    words[byte / 2] = (words[byte / 2] & ~(0xffffu << shift)) | (uint32_t(raw) << shift);
}

static void load_flux(const uint32_t* words, int count, bool splice = false) {
    flux.clear();
    cursor = 0;
    now_us = 0;
    int last = -1;
    for (int bit = 0; bit < count * 32; ++bit) {
        if ((words[bit / 32] >> (31 - bit % 32)) & 1) {
            if (last >= 0) {
                int interval = bit - last;
                CHECK(splice || (interval >= 2 && interval <= 4));
                flux.push_back(uint32_t(interval) * 100);
            }
            last = bit;
        }
    }
}

static void test_sector(const std::array<uint8_t, 512>& data) {
    std::array<uint32_t, MFM_SECTOR_WORDS + 2> words{};
    words.front() = words.back() = 0xdeadbeef;
    CHECK(mfm_encode_sector(data.data(), words.data() + 1) == MFM_SECTOR_WORDS);
    CHECK(words.front() == 0xdeadbeef && words.back() == 0xdeadbeef);
    auto encoded = words.data() + 1;
    int prev = 0;
    for (int i = 0; i < 12; ++i) CHECK(decode(raw_byte(encoded, i), prev) == 0);
    for (int i = 12; i < 15; ++i) CHECK(raw_byte(encoded, i) == 0x4489);
    prev = 1;
    CHECK(decode(raw_byte(encoded, 15), prev) == 0xfb);
    std::vector<uint8_t> covered{0xa1, 0xa1, 0xa1, 0xfb};
    for (int i = 0; i < 512; ++i) {
        CHECK(decode(raw_byte(encoded, 16 + i), prev) == data[i]);
        covered.push_back(data[i]);
    }
    uint16_t expected = crc(covered);
    CHECK(decode(raw_byte(encoded, 528), prev) == (expected >> 8));
    CHECK(decode(raw_byte(encoded, 529), prev) == (expected & 0xff));
    CHECK(decode(raw_byte(encoded, 530), prev) == 0x4e);
    CHECK(decode(raw_byte(encoded, 531), prev) == 0x4e);
    load_flux(encoded, MFM_SECTOR_WORDS);

    std::array<uint32_t, MFM_SECTOR_DATA_WORDS> raw{};
    CHECK(mfm_encode_sector_data(data.data(), raw.data()) == MFM_SECTOR_DATA_WORDS);
    for (int i = 0; i < 514; ++i)
        CHECK(raw_byte(raw.data(), i) == raw_byte(encoded, i + 16));

    // Model exactly the sector splice: unchanged ID/GAP2, new sync + DAM +
    // payload/CRC + two gap bytes. Check the real reader and neighbors.
    std::array<uint32_t, 6218> track{};
    CHECK(format_track_encode(0, 0, track.data(), int(track.size())) == int(track.size()));
    std::array<uint8_t, 512> readback{};
    for (int latency_us : {0, 3, 7, 16}) {
        CHECK(format_track_encode(0, 0, track.data(), int(track.size())) == int(track.size()));
        for (int bit = 0; bit < MFM_SECTOR_WORDS * 32; ++bit) {
            int position = 190 * 16 + latency_us + bit;
            uint32_t mask = 1u << (31 - position % 32);
            bool value = (encoded[bit / 32] >> (31 - bit % 32)) & 1;
            track[position / 32] = (track[position / 32] & ~mask) | (value ? mask : 0);
        }
        load_flux(track.data(), int(track.size()), true);
        CHECK(read_physical_sector(0, 0, 1, readback.data()));
        CHECK(readback == data);
        for (uint8_t neighbor : {2, 18}) {
            load_flux(track.data(), int(track.size()), true);
            CHECK(read_physical_sector(0, 0, neighbor, readback.data()));
            for (auto byte : readback) CHECK(byte == 0xe5);
        }
    }
}

static void test_track(int word_count) {
    std::vector<uint32_t> words(word_count + 2, 0xdeadbeef);
    CHECK(format_track_encode(79, 1, words.data() + 1, word_count) == word_count);
    CHECK(words.front() == 0xdeadbeef && words.back() == 0xdeadbeef);
    auto encoded = words.data() + 1;
    for (int i = 92; i < 95; ++i) CHECK(raw_byte(encoded, i) == 0x5224);
    for (int sec = 1; sec <= 18; ++sec) {
        int id = 146 + (sec - 1) * 658 + 12;
        for (int i = 0; i < 3; ++i) CHECK(raw_byte(encoded, id + i) == 0x4489);
        int prev = 1;
        std::vector<uint8_t> header{0xa1, 0xa1, 0xa1};
        for (int i = 3; i < 8; ++i) header.push_back(decode(raw_byte(encoded, id + i), prev));
        CHECK(header[3] == 0xfe && header[4] == 79 && header[5] == 1);
        CHECK(header[6] == sec && header[7] == 2);
        auto expected = crc(header);
        CHECK(decode(raw_byte(encoded, id + 8), prev) == (expected >> 8));
        CHECK(decode(raw_byte(encoded, id + 9), prev) == (expected & 0xff));

        load_flux(encoded, word_count);
        uint32_t boundary = 0;
        CHECK(locate_physical_sector(79, 1, uint8_t(sec), &boundary));
        // Timestamp includes at most one transition of lookahead and loop
        // overhead; it must remain within the 352 us GAP2 scheduling window.
        CHECK(boundary > 0);
        std::array<uint8_t, 512> data{};
        CHECK(read_physical_sector(79, 1, uint8_t(sec), data.data()));
        for (uint8_t byte : data) CHECK(byte == 0xe5);
    }

    // Reject wrong CHS and bad ID CRC without locating.
    load_flux(encoded, word_count);
    uint32_t boundary;
    CHECK(!locate_physical_sector(78, 1, 1, &boundary));
    int id = 146 + 12;
    int prev = 0; // size code 2 ends in zero
    auto bad = encode_byte(0x00, prev);
    replace_byte(words.data() + 1, id + 8, bad);
    replace_byte(words.data() + 1, id + 9, encode_byte(0x00, prev));
    replace_byte(words.data() + 1, id + 10, encode_byte(0x4e, prev));
    load_flux(encoded, word_count);
    CHECK(!locate_physical_sector(79, 1, 1, &boundary));

    // Valid ID but no data address mark: the next sector's identical fill
    // must not be reported as a successful read of this sector.
    CHECK(format_track_encode(79, 1, encoded, word_count) == word_count);
    prev = 1;
    replace_byte(words.data() + 1, 190 + 15, encode_byte(0xfc, prev));
    load_flux(encoded, word_count);
    std::array<uint8_t, 512> data{};
    CHECK(!read_physical_sector(79, 1, 1, data.data()));

    // CRC-valid ID with the wrong size code is unsafe for a 512-byte splice.
    CHECK(format_track_encode(79, 1, encoded, word_count) == word_count);
    prev = 1;
    std::vector<uint8_t> small{0xa1, 0xa1, 0xa1, 0xfe, 79, 1, 1, 1};
    for (int i = 3; i < 8; ++i)
        replace_byte(encoded, id + i, encode_byte(small[i], prev));
    auto small_crc = crc(small);
    replace_byte(encoded, id + 8, encode_byte(uint8_t(small_crc >> 8), prev));
    replace_byte(encoded, id + 9, encode_byte(uint8_t(small_crc), prev));
    replace_byte(encoded, id + 10, encode_byte(0x4e, prev));
    load_flux(encoded, word_count);
    CHECK(!locate_physical_sector(79, 1, 1, &boundary));
}

int main() {
    const std::vector<uint8_t> known{'1','2','3','4','5','6','7','8','9'};
    CHECK(crc(known) == 0x29b1);
    std::array<uint8_t, 512> data{};
    for (uint8_t fill : {0x00, 0xff, 0x4e, 0xe5, 0xa1, 0x55, 0xaa}) {
        data.fill(fill);
        test_sector(data);
    }
    for (int i = 0; i < 512; ++i) data[i] = uint8_t(i);
    test_sector(data);
    uint32_t seed = 0x12345678;
    for (int run = 0; run < 64; ++run) {
        for (auto& byte : data) {
            seed ^= seed << 13; seed ^= seed >> 17; seed ^= seed << 5;
            byte = uint8_t(seed);
        }
        test_sector(data);
    }
    test_track(MFM_TRACK_MIN_WORDS);
    test_track(6218); // 200 ms revolution with a 1 ms guard
    std::array<uint32_t, MFM_TRACK_MAX_WORDS> guard{};
    CHECK(format_track_encode(0, 0, guard.data(), MFM_TRACK_MIN_WORDS - 1) == -1);
    CHECK(format_track_encode(80, 0, guard.data(), MFM_TRACK_MAX_WORDS) == -1);
    CHECK(format_track_encode(0, 2, guard.data(), MFM_TRACK_MAX_WORDS) == -1);
    CHECK(mfm_encode_sector(nullptr, guard.data()) == -1);
    std::puts("MFM packing, sync marks, clock rules, CRC, bounds and real decoder tests passed");
}
