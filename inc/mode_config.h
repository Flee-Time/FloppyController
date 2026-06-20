#pragma once
#include <stdint.h>
#include <stdbool.h>

// Four DIP switches on GPIO 14, 15, 26, 27 give 16 modes.
// Bit 0 (LSB / GPIO27) = debug serial (CDC ACM)
// Bit 1       (GPIO26) = write enable (future)
// Bit 2       (GPIO15) = GreaseWeasel raw-flux mode (future)
// Bit 3 (MSB / GPIO14) = reserved

#define MODE_DEBUG_SERIAL  0x01
#define MODE_WRITE_ENABLE  0x02
#define MODE_GREASEWEASEL  0x04
#define MODE_RESERVED      0x08

#ifdef __cplusplus
extern "C" {
#endif

void mode_config_init(void);
uint8_t mode_config_get(void);
bool mode_has(uint8_t flag);

#ifdef __cplusplus
}
#endif
