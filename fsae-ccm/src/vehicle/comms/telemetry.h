// Anteater Electric Racing, 2025
#pragma once

#include <stdint.h>

#include "peripherals/adc.h"
#include "peripherals/can.h"
#include "utils/utils.h"
#include "vehicle/comms/bus.h"
#include "vehicle/devices/apps.h"
#include "vehicle/devices/bse.h"
#include "vehicle/devices/linpots.h"
#include "vehicle/devices/rtm.h"
#include "vehicle/faults.h"
#include "vehicle/vcu.h"

typedef struct __attribute__((packed)) {

    // Analog Data
    bool RTMState;

    float APPS_Travel; // APPS travel in %

    float BSEFront; // front brake pressure in PSI
    float BSERear;  // rear brake pressure in PSI
    float BSEAvg;
    float BRLinpot; // Rear Right Shock Travel in mm
    float FRLinpot; // Front Right Shock Travel in mm
    float BLLinpot; // Rear Left Shock Travel in mm
    float FLLinpot; // Front Left Shock Travel in mm

    float imdResistance;
    uint32_t imdStatus;

    // BMS Data
    float packVoltage;
    float packCurrent;
    float soc;
    float dischargeLimit;
    float chargeLimit;
    float lowCellVolt;  // Volts (e.g., 3.4215f)
    float highCellVolt; // Volts
    float avgCellVolt;  // Volts

    // MCU1 Data (DTI Essential Readings)
    uint8_t controlMode;
    // 1: CONTROL_MODE_SPEED
    // 2: CONTROL_MODE_CURRENT
    // 3: CONTROL_MODE_CURRENT_BRAKE
    // 4: CONTROL_MODE_POS
    // 7: CONTROL_MODE_NONE
    // 0, 5, 6: NOT USED
    float targetIq;
    float motorPosition;  // in degrees
    uint8_t isMotorStill; // in still position or not
    float eRPM;           // eRPM = motor RPM * number of motor pole pairs
    float dutyCycle;      // controller duty cycle
    float inputVoltage;   // dcVoltage
    float acCurrent;      // AC motor current (sign is regen or running)
    float dcCurrent;      // AC motor current (sign is regen or running)
    float controllerTemp; // temp of inverter semiconductors
    float motorTemp;      // temp of motor measured by inverter
    uint8_t faultCode;    // all inverter faults, add to faultMAP TODO
    float focId;          // foc alg Id (d-axis current in A)
    float focIq;          // foc alg Iq (q-axis current in A)

    uint8_t driveEnabled; // RTM toggle

    float maxAC_Current;
    float avMaxAC_Current;
    float minAC_Current;
    float avMinAC_Current;
    float maxDC_Current;
    float avMaxDC_Current;
    float minDC_Current;
    float avMinDC_Current;

    // MCU1 Data (DTI Non-Essential Readings)
    float dti_throttleInput; // throttle straight to inverter
    float dti_brakeInput;

    // Digital Inputs/Outputs
    bool digitalIn1;
    bool digitalIn2;
    bool digitalIn3;
    bool digitalIn4;
    bool digitalOut1;
    bool digitalOut2;
    bool digitalOut3;
    bool digitalOut4;

    // Limits Active - Group 1
    bool capTempLimitActive;
    bool dcTempLimitActive;
    bool driveEnableLimitActive;
    bool IGBTaccelLimitActive;
    bool IGBTtempLimitActive;
    bool inputVoltageLimitActive;
    bool motorAccelTempLimitActive;
    bool motorTempLimitActive;

    // Limits Active - Group 2
    bool RPMminLimitActive;
    bool RPMmaxLimitActive;
    bool powerLimitActive;

    uint8_t CANmapVersion;

    VehicleState vehicleState;

    // Dynamics Data
    uint32_t faultMap; // Debug data

} TelemetryData;

void Telemetry_Init();
void threadTelemetry(void *pvParameters);
TelemetryData const *Telemetry_GetData();
