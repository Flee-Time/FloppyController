#pragma once
#include <stdint.h>
using uint = unsigned;
uint32_t test_stall_flags();
struct TestDebugRegister {
    void operator=(uint32_t) {} // W1C, not a normal RAM register
    operator uint32_t() const { return test_stall_flags(); }
};
struct TestPio { TestDebugRegister fdebug; volatile uint32_t txf[4]; };
using PIO = TestPio*;
extern PIO pio1;
constexpr unsigned PIO_FDEBUG_TXSTALL_LSB = 24;
void pio_interrupt_clear(PIO, uint);
bool pio_interrupt_get(PIO, uint);
void pio_sm_put(PIO, uint, uint32_t);
uint pio_get_dreq(PIO, uint, bool);
bool pio_sm_is_tx_fifo_full(PIO, uint);
void pio_sm_set_enabled(PIO, uint, bool);
void pio_sm_clear_fifos(PIO, uint);
void pio_sm_set_pins_with_mask(PIO, uint, uint32_t, uint32_t);
