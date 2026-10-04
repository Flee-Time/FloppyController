#include "tusb.h"
#include "usb_task.h"

static unsigned events_left;

extern "C" bool usb_task_take_event() {
    if (!events_left) return false;
    --events_left;
    return true;
}

void usb_task_run() {
    events_left = 16;
    tud_task();
}
