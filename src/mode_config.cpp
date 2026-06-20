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

    sleep_us(100); // let pull-ups settle

    uint8_t raw = 0;
    if (!gpio_get(PIN_DIP_1)) raw |= 0x01;
    if (!gpio_get(PIN_DIP_2)) raw |= 0x02;
    if (!gpio_get(PIN_DIP_3)) raw |= 0x04;
    if (!gpio_get(PIN_DIP_4)) raw |= 0x08;
    g_mode = raw;
}

uint8_t mode_config_get(void) {
    return g_mode;
}

bool mode_has(uint8_t flag) {
    return (g_mode & flag) != 0;
}
