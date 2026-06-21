#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "mode_config.h"
#include "hw_config.h"

static uint8_t g_mode = 0;

void mode_config_init(void) {
    gpio_init(PIN_DIP_1); gpio_set_dir(PIN_DIP_1, GPIO_IN); gpio_pull_up(PIN_DIP_1);
    gpio_init(PIN_DIP_2); gpio_set_dir(PIN_DIP_2, GPIO_IN); gpio_pull_up(PIN_DIP_2);
    gpio_init(PIN_DIP_3); gpio_set_dir(PIN_DIP_3, GPIO_IN); gpio_pull_up(PIN_DIP_3);
    gpio_init(PIN_DIP_4); gpio_set_dir(PIN_DIP_4, GPIO_IN); gpio_pull_up(PIN_DIP_4);

    sleep_us(100);

    uint8_t raw = 0;
    if (!gpio_get(PIN_DIP_1)) raw |= MODE_BIT_DEBUG;
    if (!gpio_get(PIN_DIP_2)) raw |= MODE_BIT_WRITE;
    if (!gpio_get(PIN_DIP_3)) raw |= MODE_BIT_OPMODE;
    if (!gpio_get(PIN_DIP_4)) raw |= MODE_BIT_RESERVED;
    g_mode = raw;
}

uint8_t mode_config_get(void) {
    return g_mode;
}

bool mode_has_debug(void) {
    return (g_mode & MODE_BIT_DEBUG) != 0;
}

bool mode_has_write(void) {
    return (g_mode & MODE_BIT_WRITE) != 0;
}

bool mode_has_gw(void) {
    return (g_mode & MODE_BIT_OPMODE) != 0;
}
