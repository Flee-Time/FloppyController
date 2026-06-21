#include "tusb.h"
#include "mode_config.h"
#include "pico/unique_id.h"

tusb_desc_device_t const desc_device_msc = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0xCafe,
    .idProduct          = 0x4002,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

tusb_desc_device_t const desc_device_gw = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x1209,
    .idProduct          = 0x4d69,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void) {
    return mode_has_gw()
        ? (uint8_t const *) &desc_device_gw
        : (uint8_t const *) &desc_device_msc;
}

#define EPNUM_MSC_OUT     0x01
#define EPNUM_MSC_IN      0x81
#define EPNUM_CDC_NOTIF   0x82
#define EPNUM_CDC_OUT     0x03
#define EPNUM_CDC_IN      0x83

// Full: MSC + CDC
#define FULL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN + TUD_CDC_DESC_LEN)
enum { FULL_MSC=0, FULL_CDC_COMM, FULL_CDC_DATA, FULL_TOTAL };

uint8_t const desc_full[] = {
    TUD_CONFIG_DESCRIPTOR(1, FULL_TOTAL, 0, FULL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),
    TUD_MSC_DESCRIPTOR(FULL_MSC, 0, EPNUM_MSC_OUT, EPNUM_MSC_IN, 64),
    TUD_CDC_DESCRIPTOR(FULL_CDC_COMM, 0, EPNUM_CDC_NOTIF,
                       8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

// GW: CDC only
#define GW_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)
enum { GW_CDC_COMM=0, GW_CDC_DATA, GW_TOTAL };

uint8_t const desc_gw[] = {
    TUD_CONFIG_DESCRIPTOR(1, GW_TOTAL, 0, GW_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),
    TUD_CDC_DESCRIPTOR(GW_CDC_COMM, 0, EPNUM_CDC_NOTIF,
                       8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return mode_has_gw() ? desc_gw : desc_full;
}

// --- String Descriptors ---
static const char* str_product_m = "DualCore Floppy (gw-compat)";
static const char* str_product_f = "DualCore Floppy";
static const char* str_serial   = nullptr;

static char serial_buf[17];

static void make_serial(void) {
    if (str_serial) return;
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    for (int i = 0; i < 8; i++) {
        static const char hex[] = "0123456789ABCDEF";
        serial_buf[i*2]     = hex[(id.id[i] >> 4) & 0xF];
        serial_buf[i*2 + 1] = hex[id.id[i] & 0xF];
    }
    serial_buf[16] = 0;
    str_serial = serial_buf;
}

char const* string_desc_arr [] = {
    (const char[]) { 0x09, 0x04 },
    "FleeTime",
    nullptr, // product string set dynamically
    nullptr, // serial set dynamically
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (!(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0]))) return NULL;

        const char* str;
        if (index == 2) {
            str = mode_has_gw() ? str_product_m : str_product_f;
        } else if (index == 3) {
            make_serial();
            str = str_serial;
        } else {
            str = string_desc_arr[index];
        }
        if (!str) return NULL;

        chr_count = strlen(str);
        if (chr_count > 31) chr_count = 31;

        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1+i] = str[i];
        }
    }

    _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);
    return _desc_str;
}
