#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// --- System ---
#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE
#define CFG_TUSB_OS             OPT_OS_PICO

// --- Memory ---
#define CFG_TUD_MAINTASK_SIZE   2048
#define CFG_TUD_ENDPOINT0_SIZE  64

// --- Enabled Classes ---
#define CFG_TUD_MSC             1

// --- MSC Configuration ---
#define CFG_TUD_MSC_EP_BUFSIZE  512

#ifdef __cplusplus
}
#endif