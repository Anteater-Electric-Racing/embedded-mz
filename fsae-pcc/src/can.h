// Anteater Electric Racing, 2025

#pragma once

#include <FreeRTOS.h>
#include <stdint.h>

#define PCC_CAN_ID 0x222
#define TEMP_CAN_ID 0x223

// bms can timeout - using inline so it is defined at compile time
inline constexpr uint32_t BMS_CAN_TIMEOUT_MS = 1500;

typedef struct __attribute__((packed)) {
    uint8_t state;               // Precharge state
    uint8_t errorCode;           // Error code
    uint16_t accumulatorVoltage; // Accumulator voltage in volts
    uint16_t tsVoltage;          // Transmission side voltage in volts
    uint16_t prechargeProgress;  // Precharge progress in percent
} PCCData;

typedef struct __attribute__((packed)) {
    int16_t T1Temp; // Thermistor 1 Temperature
    int16_t T2Temp; // Thermistor 2 Temperature
    uint8_t isSafeTemperature : 1;
} PCCTempData;

void CAN_Init();

void canSendMessage(uint32_t id, const void *data, uint8_t length);

bool CAN_IsChargerSafetyActive();
// get last bms can rx time - using Ticktype_t instead of uint32_t from freeRTOS
TickType_t CAN_GetBMSLastRxTime();
float CAN_GetChargerVoltage();
float CAN_GetChargerCCL();
uint8_t CAN_GetChargerCounter();
