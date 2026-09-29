#include "regen.h"

#include <algorithm>
#include <cmath>

namespace {
float fraction(float value, float start, float end) {
    return std::max(0.0f, std::min((value - start) / (end - start), 1.0f));
}
} // namespace

RegenRequest Regen_Calculate(const RegenInputs &in, const RegenConfig &c) {
    // 1. Reject missing calibration or invalid readings.
    const float values[] = {in.accelerator,     in.brakePsi,
                            in.packV,           in.batteryLowC,
                            in.batteryHighC,    in.chargeLimitA,
                            in.forwardErpm,     c.liftOffEnd,
                            in.brakeRestPsi,    c.brakeFullAboveRestPsi,
                            c.liftOffA,         c.brakeAddedA,
                            c.maxMotorA,        c.maxBatteryA,
                            c.voltageStartV,    c.voltageStopV,
                            c.batteryMinC,      c.temperatureStartC,
                            c.temperatureStopC, c.speedStartErpm,
                            c.speedFullErpm};
    for (float value : values) {
        if (!std::isfinite(value))
            return {};
    }
    if (c.liftOffEnd <= 0 || c.liftOffEnd >= 1 ||
        c.brakeFullAboveRestPsi <= 0 || c.liftOffA < 0 || c.brakeAddedA < 0 ||
        c.maxMotorA <= 0 || c.maxBatteryA <= 0 || c.voltageStartV <= 0 ||
        c.voltageStopV <= c.voltageStartV ||
        c.temperatureStartC <= c.batteryMinC ||
        c.temperatureStopC <= c.temperatureStartC || c.speedStartErpm < 0 ||
        c.speedFullErpm <= c.speedStartErpm || in.accelerator < 0 ||
        in.accelerator > 1 || in.packV <= 0 ||
        in.batteryLowC > in.batteryHighC || in.batteryLowC < c.batteryMinC ||
        in.chargeLimitA <= 0)
        return {};

    // 2. Add mild lift-off braking and pressure-dependent braking.
    const float liftOff = 1.0f - fraction(in.accelerator, 0, c.liftOffEnd);
    // Start at the calibrated rest pressure; full demand is rest + offset.
    const float brake =
        fraction(in.brakePsi - in.brakeRestPsi, 0, c.brakeFullAboveRestPsi);
    const float requestedA =
        std::min(c.liftOffA * liftOff + c.brakeAddedA * brake, c.maxMotorA);

    // 3. Reduce regen as pack voltage or battery temperature rises.
    const float voltageFactor =
        1.0f - fraction(in.packV, c.voltageStartV, c.voltageStopV);
    const float temperatureFactor =
        1.0f -
        fraction(in.batteryHighC, c.temperatureStartC, c.temperatureStopC);
    const float batteryFactor = std::min(voltageFactor, temperatureFactor);
    const float speedFactor =
        fraction(in.forwardErpm, c.speedStartErpm, c.speedFullErpm);

    // Motor AC amps and battery DC amps are different limits. A live DTI
    // adapter must enforce BOTH; do not use the BMS limit as a motor-amp cap.
    RegenRequest out;
    out.motorBrakeA = requestedA * std::min(batteryFactor, speedFactor);
    out.batteryChargeLimitA =
        std::min(in.chargeLimitA, c.maxBatteryA) * batteryFactor;
    return out;
}
