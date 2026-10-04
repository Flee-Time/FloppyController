#pragma once
#include <stdint.h>
uint32_t save_and_disable_interrupts();
void restore_interrupts(uint32_t);
