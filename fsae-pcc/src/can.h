// Anteater Electric Racing, 2025

#pragma once

#include <stdint.h>

void CAN_Init();
void CAN_SendPCCMessage(uint8_t state, uint8_t errorCode,
                        float accumulatorVoltage, float tsVoltage,
                        float prechargeProgress);

void CAN_PollMessages();
bool CAN_IsChargerSafetyActive();
uint32_t CAN_GetBMSLastRxTime();
float CAN_GetChargerVoltage();
float CAN_GetChargerCCL();
uint8_t CAN_GetChargerCounter();
