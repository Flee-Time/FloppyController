#pragma once
struct TestQueue {};
using osal_queue_t = TestQueue*;
extern "C" bool osal_queue_receive(osal_queue_t, void*, uint32_t);
