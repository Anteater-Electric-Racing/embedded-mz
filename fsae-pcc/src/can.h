// Anteater Electric Racing, 2025

#pragma once

#include <FreeRTOS.h>
#include <stdint.h>

// bms can timeout - using inline so it is defined at compile time
inline constexpr uint32_t BMS_CAN_TIMEOUT_MS = 1500;

void CAN_Init();
void CAN_SendPCCMessage(uint8_t state, uint8_t errorCode,
                        float accumulatorVoltage, float tsVoltage,
                        float prechargeProgress);

bool CAN_IsChargerSafetyActive();
// get last bms can rx time - using Ticktype_t instead of uint32_t from freeRTOS
TickType_t CAN_GetBMSLastRxTime();
float CAN_GetChargerVoltage();
float CAN_GetChargerCCL();
uint8_t CAN_GetChargerCounter();
