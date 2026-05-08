// TO REOMVE

#pragma once

#include <stdint.h>

typedef struct __attribute__((packed)) {
    uint8_t state;
    uint8_t errorCode;
    uint16_t accumulatorVoltage;
    uint16_t tsVoltage;
    uint16_t prechargeProgress;
} PCC;

void PCC_Init();
void PCC_ProcessPCCMessage(uint64_t *rx_data);
PCC *PCC_GetData();

bool PCC_PrechargeComplete();
