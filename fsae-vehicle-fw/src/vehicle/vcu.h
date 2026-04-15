// Anteater Electric Racing, 2025

#pragma once

#include "vehicle/devices/dti.h"
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
typedef struct {
    DriveStrategy driveStrategy;
    DTIControlMode controlMode;
} DriveState;

void VCU_Init();
void threadVCU(void *pvParameters);
void VCU_SetFaultState();
void VCU_ClearFaultState();
void VCU_ForceIdleState();
float derate();
float VCU_TorqueMap(float pedal);

VehicleState VCU_GetState();
