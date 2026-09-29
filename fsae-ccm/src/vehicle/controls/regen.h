#pragma once
#include "utils/utils.h"
#include "vehicle/devices/bse.h"

// Zero defaults disable the preview until vehicle calibration is filled in.
struct RegenConfig {
    float liftOffEnd = 0.25f; // accelerator fraction where lift-off regen ends
    float brakeFullAboveRestPsi = BSE_VOLTAGE_TO_PSI(
        BSE_GetBSEAverage()); // PSI rise for full added regen. Must be
                              // initialized before Brakes are pressed down
    float liftOffA = 0.0f;    // motor AC amps at released accelerator
    float brakeAddedA = 0.0f; // extra motor AC amps at full brake pressure
    float maxMotorA = 0.0f;
    float maxBatteryA = 0.0f; // battery DC charging amps, not motor amps
    float voltageStartV = 290.0f;
    float voltageStopV = 400.0f;
    float batteryMinC = 0.0f; // no regen below this temperature
    float temperatureStartC = 0.0f;
    float temperatureStopC = 0.0f;
    float speedStartErpm = 20000.0f;
    float speedFullErpm = 80000.0f;
};

struct RegenInputs {
    float accelerator = 0.0f;  // normalized 0..1
    float brakePsi = 0.0f;     // current average brake pressure
    float brakeRestPsi = 0.0f; // average from startup calibration
    float packV = 0.0f;
    float batteryLowC = 0.0f;
    float batteryHighC = 0.0f;
    float chargeLimitA = 0.0f; // BMS charge-current limit, DC amps
    float forwardErpm = 0.0f;  // positive for forward motion
};

struct RegenRequest {
    float motorBrakeA = 0.0f;
    float batteryChargeLimitA = 0.0f;
};

// Preview only. Does not send CAN, select drive/brake mode, or validate
// telemetry age. Both returned current limits are required for live control.
RegenRequest Regen_Calculate(const RegenInputs &in, const RegenConfig &config);
