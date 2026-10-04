#pragma once
#include <stdbool.h>
#include <stdint.h>

// Keep the Pico SDK's locks and IRQ-safe queue. Only event dispatch is
// bounded: MSC return-zero retries otherwise keep tud_task() in its loop
// until the entire disk command finishes, starving application polling.
#define osal_queue_receive pico_osal_queue_receive
#include "osal/osal_pico.h"
#undef osal_queue_receive

#ifdef __cplusplus
extern "C" {
#endif
bool usb_task_take_event();

static inline bool osal_queue_receive(osal_queue_t queue, void* data, uint32_t timeout_ms) {
    return usb_task_take_event() && pico_osal_queue_receive(queue, data, timeout_ms);
}
#ifdef __cplusplus
}
#endif
