#include "led_drv.h"
#include "hw_config.h"
#include "hardware/pio.h"
#include "ws2812.pio.h" // Auto-generated PIO header

// Internal state hidden from the rest of the program
static PIO led_pio = pio1; // Reserve PIO1 exclusively for UI
static uint led_sm;

void led_init() {
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 0);

    led_sm = pio_claim_unused_sm(led_pio, true);
    uint led_offset = pio_add_program(led_pio, &ws2812_program);
    
    // Initialize the PIO using the macro from our config file
    ws2812_program_init(led_pio, led_sm, led_offset, PIN_LED, 800000, false);
    
    // Default to off on boot
    led_set_rgb(0, 0, 0);
}

void led_set_rgb(uint8_t red, uint8_t green, uint8_t blue) {
    // RGB ordering
    uint32_t mask = (red << 16) | (green << 8) | (blue << 0);
    pio_sm_put_blocking(led_pio, led_sm, mask << 8u);
}