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

constexpr float KP = 50.0f; // PLEASE TUNE THESE VALUES
constexpr float KI = 5.0f;
constexpr float KD = 0.0f;
constexpr float rpmConversion = 0.3f;
constexpr float wheelRadius = 0.5f;
constexpr float INTEGRAL_MAX = 20.0f;
constexpr float INTEGRAL_MIN = -20.0f;

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
float VCU_Derate(float temperature);
float VCU_TorqueMap(float pedal);

VehicleState VCU_GetState();
