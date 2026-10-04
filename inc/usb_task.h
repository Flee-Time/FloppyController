#pragma once

// Service a bounded batch of USB events, then let application work run.
// All callers, including GreaseWeasel's transfer loops, use this entry point.
void usb_task_run();
