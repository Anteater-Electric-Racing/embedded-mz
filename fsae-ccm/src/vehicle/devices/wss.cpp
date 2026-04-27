// Anteater Electric Racing, 2025

#include "wss.h"
#include <Arduino.h>
#include <arduino_freertos.h>
using namespace arduino;

static const float WHEEL_CIRCUMFERENCE_M = 1.6; // TODO
static const int PULSES_PER_REV = 12;           // TODO
static const float MPS_TO_MPH = 2.23694f;
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
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt_ms = currentTime - lastCalcTime;

    noInterrupts();

    unsigned int currentPulses1 = pulsesWheel1;
    unsigned int currentPulses2 = pulsesWheel2;
    pulsesWheel1 = 0;
    pulsesWheel2 = 0;

    interrupts();

    currentRPM1 = ((float)currentPulses1 / PULSES_PER_REV) * (60000.0f / dt_ms);
    currentRPM2 = ((float)currentPulses2 / PULSES_PER_REV) * (60000.0f / dt_ms);

    currentSpeedMPS1 = (currentRPM1 / 60.0f) * WHEEL_CIRCUMFERENCE_M;
    currentSpeedMPS2 = (currentRPM2 / 60.0f) * WHEEL_CIRCUMFERENCE_M;

    lastCalcTime = currentTime;
}

float WSS_GetRPM1() { return currentRPM1; }
float WSS_GetRPM2() { return currentRPM2; }

float WSS_GetSpeed1_MPH() { return currentSpeedMPS1 * MPS_TO_MPH; }
float WSS_GetSpeed2_MPH() { return currentSpeedMPS2 * MPS_TO_MPH; }
