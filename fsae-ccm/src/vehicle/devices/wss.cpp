// Anteater Electric Racing, 2025

#include "wss.h"
#include <Arduino.h>

static const float WHEEL_CIRCUMFERENCE_M = 1.6; // TODO
static const int PULSES_PER_REV = 12;           // TODO
static const unsigned long CALC_INTERVAL_MS = 100;

static volatile unsigned int pulsesWheel1 = 0;
static volatile unsigned int pulsesWheel2 = 0;

static unsigned long lastCalcTime = 0;

static float currentRPM1 = 0.0f;
static float currentRPM2 = 0.0f;
static float currentSpeedMPS1 = 0.0f;
static float currentSpeedMPS2 = 0.0f;

static void ISR_Wheel1() { ++pulsesWheel1; }
static void ISR_Wheel2() { ++pulsesWheel2; }

void WSS_Init() {
    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), ISR_Wheel1, FALLING);

    pinMode(3, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(3), ISR_Wheel2, FALLING);
}

void WSS_Update() {
    unsigned long currentTime = millis();

    if (currentTime - lastCalcTime >= CALC_INTERVAL_MS) {
        noInterrupts();

        unsigned int currentPulses1 = pulsesWheel1;
        unsigned int currentPulses2 = pulsesWheel2;
        pulsesWheel1 = 0;
        pulsesWheel2 = 0;

        interrupts();

        currentRPM1 = ((float)currentPulses1 / PULSES_PER_REV) *
                      (60000.0f / CALC_INTERVAL_MS);
        currentRPM2 = ((float)currentPulses2 / PULSES_PER_REV) *
                      (60000.0f / CALC_INTERVAL_MS);

        currentSpeedMPS1 = (currentRPM1 / 60.0f) * WHEEL_CIRCUMFERENCE_M;
        currentSpeedMPS2 = (currentRPM2 / 60.0f) * WHEEL_CIRCUMFERENCE_M;

        lastCalcTime = currentTime;
    }
}

float WSS_GetRPM1() { return currentRPM1; }
float WSS_GetRPM2() { return currentRPM2; }

float WSS_GetSpeed1_MPH() { return currentSpeedMPS1 * 2.23694f; }
float WSS_GetSpeed2_MPH() { return currentSpeedMPS2 * 2.23694f; }
