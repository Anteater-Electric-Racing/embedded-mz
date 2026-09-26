#include "wss.h"
#include <Arduino.h>

static const float WHEEL_CIRCUMFERENCE_M = 1.6f;
static const float MPS_TO_MPH = 2.23694f;
static const int PULSES_PER_REV = 12;

// Timeout threshold: If no pulse for 250ms, wheel is moving ultra slow or
// stopped(< 3 mph)

static const unsigned long TIMEOUT_US = 250000;

// Structure to isolate ISR data variables per wheel
struct WheelISRData {
    volatile unsigned long lastPulseTimeUs;
    volatile unsigned long pulsePeriodUs;
    volatile bool newPulseDetected;
};

// Global state tracking for calculated telemetry
struct WheelTelemetry {
    float currentRPM;
    float currentSpeedMPH;
};

// Arrays to handle all 4 wheels cleanly
static WheelISRData wheelISR[4] = {0};
static WheelTelemetry wheelData[4] = {0};

// --- Interrupt Service Routines ---
static void ISR_Wheel1() {
    unsigned long currentTimeUs = micros();
    unsigned long delta = currentTimeUs - wheelISR[0].lastPulseTimeUs;
    if (delta > 500) { // Hardware Debounce
        wheelISR[0].pulsePeriodUs = delta;
        wheelISR[0].lastPulseTimeUs = currentTimeUs;
        wheelISR[0].newPulseDetected = true;
    }
}

static void ISR_Wheel2() {
    unsigned long currentTimeUs = micros();
    unsigned long delta = currentTimeUs - wheelISR[1].lastPulseTimeUs;
    if (delta > 500) {
        wheelISR[1].pulsePeriodUs = delta;
        wheelISR[1].lastPulseTimeUs = currentTimeUs;
        wheelISR[1].newPulseDetected = true;
    }
}

static void ISR_Wheel3() {
    unsigned long currentTimeUs = micros();
    unsigned long delta = currentTimeUs - wheelISR[2].lastPulseTimeUs;
    if (delta > 500) {
        wheelISR[2].pulsePeriodUs = delta;
        wheelISR[2].lastPulseTimeUs = currentTimeUs;
        wheelISR[2].newPulseDetected = true;
    }
}

static void ISR_Wheel4() {
    unsigned long currentTimeUs = micros();
    unsigned long delta = currentTimeUs - wheelISR[3].lastPulseTimeUs;
    if (delta > 500) {
        wheelISR[3].pulsePeriodUs = delta;
        wheelISR[3].lastPulseTimeUs = currentTimeUs;
        wheelISR[3].newPulseDetected = true;
    }
}

void WSS_Init() {
    unsigned long startupTime = micros();

    // Explicitly map pins 23 down to 20
    const uint8_t wssPins[4] = {23, 22, 21, 20};

    // Function pointer array matching our ISRs to their indices
    void (*isrFunctions[4])() = {ISR_Wheel1, ISR_Wheel2, ISR_Wheel3,
                                 ISR_Wheel4};

    for (int i = 0; i < 4; i++) {
        pinMode(wssPins[i],
                INPUT); // Maintained standard INPUT due to external4V bias
        wheelISR[i].lastPulseTimeUs = startupTime;
        attachInterrupt(digitalPinToInterrupt(wssPins[i]), isrFunctions[i],
                        RISING);
    }
}

void WSS_Update() {
    unsigned long now = micros();

    for (int i = 0; i < 4; i++) {
        unsigned long period = 0;
        bool updated = false;

        // Atomic harvest window per channel
        noInterrupts();
        if (wheelISR[i].newPulseDetected) {
            period = wheelISR[i].pulsePeriodUs;
            wheelISR[i].newPulseDetected = false;
            updated = true;
        }
        interrupts();

        if (updated) {
            // High-resolution conversion calculation
            wheelData[i].currentRPM =
                60.0f / ((float)PULSES_PER_REV * ((float)period / 1000000.0f));
            float speedMPS =
                (wheelData[i].currentRPM / 60.0f) * WHEEL_CIRCUMFERENCE_M;
            wheelData[i].currentSpeedMPH = speedMPS * MPS_TO_MPH;
        } else if (now - wheelISR[i].lastPulseTimeUs > TIMEOUT_US) {
            // Handle active timeout decay for individual stopped wheels
            wheelData[i].currentRPM = 0.0f;
            wheelData[i].currentSpeedMPH = 0.0f;
        }
    }
}

// --- Telemetry Getters ---
float WSS_GetRPM1() { return wheelData[0].currentRPM; }
float WSS_GetRPM2() { return wheelData[1].currentRPM; }
float WSS_GetRPM3() { return wheelData[2].currentRPM; }
float WSS_GetRPM4() { return wheelData[3].currentRPM; }

float WSS_GetSpeed1_MPH() { return wheelData[0].currentSpeedMPH; }
float WSS_GetSpeed2_MPH() { return wheelData[1].currentSpeedMPH; }
float WSS_GetSpeed3_MPH() { return wheelData[2].currentSpeedMPH; }
float WSS_GetSpeed4_MPH() { return wheelData[3].currentSpeedMPH; }
