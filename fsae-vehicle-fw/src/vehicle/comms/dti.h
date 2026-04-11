// Anteater Electric Racing, 2025

#pragma once

#include <stdint.h>

#define dtiNodeID 0x65 // TODO
typedef enum {
    setCurrent_ID = 0x01,
    setBrakeCurrent_ID = 0x02,
    setERPM_ID = 0x03,
    setPosition_ID = 0x04,
    setRelativeCurrent_ID = 0x05,
    setRelativeBrakeCurrent_ID = 0x06,
    setDigIO_ID = 0x07,
    setMaxCurrentAC_ID = 0x08,
    setMaxBrakeCurrentAC_ID = 0x09,
    setMaxCurrentDC_ID = 0x0A,
    setMaxBrakeCurrentDC_ID = 0x0B,
    setDriveEnable_ID = 0x0C
} DTI_Send;

// include a LUT for throttle values

typedef enum {
    MODE_AC_FORWARD,
    MODE_AC_ALL,
    MODE_SPEED,
    MODE_FOC,
    MODE_TC,
    MODE_COMP
} ControlMode;

void threadMotor(void *pvParameters);

void Motor_Init();
void Motor_UpdateMotor(float torqueDemand);
void Motor_UpdateMotor(float torqueDemand, bool enablePrecharge,
                       bool enablePower, bool enableRun, bool enableRegen,
                       bool enableStandby);
float torqueMap(float pedal);
float Motor_GetTorqueDemand();
ControlMode DTI_GetControlMode();
void threadMotor(void *pvParameters);

void Motor_SetFaultState();
void Motor_ClearFaultState();
void Motor_ClearToIdleFault();
float Motor_TargetTorque();
