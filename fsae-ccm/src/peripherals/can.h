// Anteater Electric Racing, 2025

#pragma once

#include "vehicle/devices/dti.h"
#include <stdint.h>

// struct DTIMessage; // no circular defintion

void CAN_Init();
void CAN_Send(uint32_t id, uint64_t msg);

void CAN_Send(DTIMessage *msg);

void CAN_Receive(uint32_t *rx_id, uint64_t *rx_data);

// Standard Classic CAN on bus 2 or 3. False means invalid input or queue full.
bool CAN_SendFrame(uint8_t bus, uint32_t id, const uint8_t *data, uint8_t length);

bool CAN_IsBusHealthy(uint8_t bus);
void CAN_CheckHealth();
