#pragma once
#include <stdint.h>
#include <stdbool.h>

// Four DIP switches on GPIO 14, 15, 26, 27 give 16 modes.
// Bit 0 (LSB / GPIO27 / DIP1) = Debug Serial (CDC ACM output)
// Bit 1       (GPIO26 / DIP2) = Write Enable
// Bit 2       (GPIO15 / DIP3) = Operation Mode (0 = MSC, 1 = GreaseWeasel)
// Bit 3 (MSB / GPIO14 / DIP4) = Reserved
//
// | Mode | DIP4 DIP3 DIP2 DIP1 | Name                    |
// |------|---------------------|-------------------------|
// |  0   | OFF  OFF  OFF  OFF  | MSC only                |
// |  1   | OFF  OFF  OFF  ON   | MSC + Debug             |
// |  2   | OFF  OFF  ON   OFF  | MSC + Write             |
// |  3   | OFF  OFF  ON   ON   | MSC + Debug + Write     |
// |  4   | OFF  ON   OFF  OFF  | GreaseWeasel only       |
// |  5   | OFF  ON   OFF  ON   | GreaseWeasel + Debug    |
// |  6   | OFF  ON   ON   OFF  | GreaseWeasel + Write    |
// |  7   | OFF  ON   ON   ON   | GreaseWeasel + Debug + Write |
// | 8-15 | ON   *    *    *    | Reserved                |
//
// DIP switches are active-low (ON = grounded = logic 0 at GPIO = 1 in mode byte).

#define MODE_MSC                0x00
#define MODE_MSC_DEBUG          0x01
#define MODE_MSC_WRITE          0x02
#define MODE_MSC_DEBUG_WRITE    0x03
#define MODE_GW                 0x04
#define MODE_GW_DEBUG           0x05
#define MODE_GW_WRITE           0x06
#define MODE_GW_DEBUG_WRITE     0x07

#define MODE_BIT_DEBUG          0x01
#define MODE_BIT_WRITE          0x02
#define MODE_BIT_OPMODE         0x04
#define MODE_BIT_RESERVED       0x08

#ifdef __cplusplus
extern "C" {
#endif

void mode_config_init(void);
uint8_t mode_config_get(void);
bool mode_has_debug(void);
bool mode_has_write(void);
bool mode_has_gw(void);

#ifdef __cplusplus
}
#endif
