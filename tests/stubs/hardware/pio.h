#pragma once
#include <stdint.h>
using uint = unsigned;
using PIO = void*;
void pio_sm_clear_fifos(PIO, uint);
bool pio_sm_is_rx_fifo_empty(PIO, uint);
uint32_t pio_sm_get(PIO, uint);
