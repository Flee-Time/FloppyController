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
#define CFG_TUD_CDC             1

// --- MSC Configuration ---
#define CFG_TUD_MSC_EP_BUFSIZE  512

// --- CDC Configuration ---
#define CFG_TUD_CDC_RX_BUFSIZE  512
#define CFG_TUD_CDC_TX_BUFSIZE  512

// --- Device ---
#define CFG_TUD_TASK_QUEUE_SZ    16

#ifdef __cplusplus
}
#endif
