// Anteater Electric Racing

#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifndef FAULT_LOG_CAPACITY
#define FAULT_LOG_CAPACITY                                                     \
    20 // 5 bytes/entry -> bump this one number for more room
#endif

// 5 bytes, packed.
typedef struct __attribute__((packed)) {
    uint32_t timestamp_ms;
    uint8_t bit; // which bit (0-31) turned on
} FaultLogEntry_t;

void FaultLog_Init(void);

void FaultLog_CheckBits(uint32_t currentBits);

uint16_t FaultLog_Count(void);                           // valid entries stored
bool FaultLog_Get(uint16_t index, FaultLogEntry_t *out); // 0 = oldest
void FaultLog_Clear(void);

// Prints every stored entry as "<ms>: bit <n>", oldest first.
void FaultLog_Print(void);