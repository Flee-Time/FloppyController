#include "tusb_os_custom.h"
#include "usb_task.h"
#include <cstdio>
#include <cstdlib>
#include <deque>

#define CHECK(expr) do { if (!(expr)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); std::exit(1); \
} } while (0)

static TestQueue queue;
static std::deque<int> events;
static unsigned dispatched, cdc_events, reset_events, app_polls;
static bool busy = true;

extern "C" bool pico_osal_queue_receive(osal_queue_t q, void* data, uint32_t timeout) {
    CHECK(q == &queue && timeout == 0);
    if (events.empty()) return false;
    *static_cast<int*>(data) = events.front();
    events.pop_front();
    return true;
}

void tud_task() {
    int event;
    unsigned this_run = 0;
    // Match TinyUSB's queue-draining loop. A busy MSC callback continually
    // queues another retry; application work must still get a chance to run.
    while (osal_queue_receive(&queue, &event, 0)) {
        CHECK(++this_run <= 16);
        ++dispatched;
        if (event == 1 && busy) events.push_back(1);
        if (event == 2) ++cdc_events;
        if (event == 3) ++reset_events;
    }
}

int main() {
    events.push_back(1);
    for (int i = 0; i < 1000; ++i) {
        unsigned before = dispatched;
        events.push_back(2);
        events.push_back(3);
        usb_task_run();
        CHECK(dispatched - before == 16);
        ++app_polls;
        CHECK(events.size() == 1 && events.front() == 1);
    }
    CHECK(app_polls == 1000 && cdc_events == 1000 && reset_events == 1000);
    busy = false;
    usb_task_run();
    CHECK(events.empty());
    events.push_back(2);
    usb_task_run();
    CHECK(cdc_events == 1001); // Every entry starts a new event budget.
    std::puts("Continuous MSC retries yield to application polling; CDC/reset events remain serviced");
}
