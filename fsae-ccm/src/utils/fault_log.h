// Anteater Electric Racing
// Fully generic bit-change log. This file knows nothing about DTI, CAN,
// or any other module -- it only tracks a bitmask handed to it each call
// and logs (bit position, timestamp) for every bit that turns 0->1.
// Whoever calls FaultLog_CheckBits() is responsible for deciding what
// each bit means.

#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifndef FAULT_LOG_CAPACITY
#define FAULT_LOG_CAPACITY 20   // 5 bytes/entry -> bump this one number for more room
#endif

// 5 bytes, packed.
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t  bit;   // which bit (0-31) turned on
} FaultLogEntry_t;

void FaultLog_Init(void);

// Call every loop with the CURRENT bitmask of whatever the caller is
// tracking. Diffs against the bitmask from the last call; every bit that
// is newly set (0->1) gets its own log entry. Bits turning off, or
// staying the same either way, do nothing.
void FaultLog_CheckBits(uint32_t currentBits);

uint16_t FaultLog_Count(void);                          // valid entries stored
bool FaultLog_Get(uint16_t index, FaultLogEntry_t *out); // 0 = oldest
void FaultLog_Clear(void);

// Prints every stored entry as "<ms>: bit <n>", oldest first.
void FaultLog_Print(void);