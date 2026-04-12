// Anteater Electric Racing, 2025

#pragma once

#include <stdint.h>

// include a LUT for throttle values

typedef enum {
    STATE_OFF,
    STATE_STANDBY,
    STATE_PRECHARGING,
    STATE_IDLE,
    STATE_DRIVING,
    STATE_FAULT
} VehicleState;

typedef enum { OPEN_LOOP, TRACTION_CTRL, LAUNCH_CTRL } DriveStrategy;

void VCU_Init();
void VCU_ThreadVCU(void *pvParameters);
void VCU_SetFaultState();
void VCU_ClearFaultState();
void VCU_ForceIdleState();
void VCU_TorqueMap();
