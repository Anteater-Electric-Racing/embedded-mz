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
static volatile unsigned int pulsesWheel3 = 0;
static volatile unsigned int pulsesWheel4 = 0;

static unsigned long lastCalcTime = 0;

static float currentRPM1 = 0.0f;
static float currentRPM2 = 0.0f;
static float currentRPM3 = 0.0f;
static float currentRPM4 = 0.0f;
static float currentSpeedMPS1 = 0.0f;
static float currentSpeedMPS2 = 0.0f;
static float currentSpeedMPS3 = 0.0f;
static float currentSpeedMPS4 = 0.0f;

static void ISR_Wheel1() { ++pulsesWheel1; }
static void ISR_Wheel2() { ++pulsesWheel2; }
static void ISR_Wheel3() { ++pulsesWheel3; }
static void ISR_Wheel4() { ++pulsesWheel4; }

void WSS_Init() {
    // pinMode(23, INPUT_PULLDOWN);
    // pinMode(22, INPUT_PULLDOWN);
    // pinMode(21, INPUT_PULLDOWN);
    // pinMode(20, INPUT_PULLDOWN);

    /*wheel speed itself is pullup or pulldown already */
    // pinMode(23, INPUT);
    // pinMode(22, INPUT);
    // pinMode(21, INPUT);
    // pinMode(20, INPUTN);

    attachInterrupt(digitalPinToInterrupt(23), ISR_Wheel1, FALLING);
    attachInterrupt(digitalPinToInterrupt(22), ISR_Wheel2, FALLING);
    attachInterrupt(digitalPinToInterrupt(21), ISR_Wheel3, FALLING);
    attachInterrupt(digitalPinToInterrupt(20), ISR_Wheel4, FALLING);
}

void WSS_Update() {
    unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
    float dt_ms = currentTime - lastCalcTime;

    taskENTER_CRITICAL();

    unsigned int currentPulses1 = pulsesWheel1;
    unsigned int currentPulses2 = pulsesWheel2;
    unsigned int currentPulses3 = pulsesWheel3;
    unsigned int currentPulses4 = pulsesWheel4;

    pulsesWheel1 = 0;
    pulsesWheel2 = 0;
    pulsesWheel3 = 0;
    pulsesWheel4 = 0;

    taskEXIT_CRITICAL();

    currentRPM1 = ((float)currentPulses1 / PULSES_PER_REV) * (60000.0f / dt_ms);
    currentRPM2 = ((float)currentPulses2 / PULSES_PER_REV) * (60000.0f / dt_ms);
    currentRPM3 = ((float)currentPulses3 / PULSES_PER_REV) * (60000.0f / dt_ms);
    currentRPM4 = ((float)currentPulses4 / PULSES_PER_REV) * (60000.0f / dt_ms);

    currentSpeedMPS1 = (currentRPM1 / 60.0f) * WHEEL_CIRCUMFERENCE_M;
    currentSpeedMPS2 = (currentRPM2 / 60.0f) * WHEEL_CIRCUMFERENCE_M;
    currentSpeedMPS3 = (currentRPM3 / 60.0f) * WHEEL_CIRCUMFERENCE_M;
    currentSpeedMPS4 = (currentRPM4 / 60.0f) * WHEEL_CIRCUMFERENCE_M;

    lastCalcTime = currentTime;
}

float WSS_GetRPM1() { return currentRPM1; }
float WSS_GetRPM2() { return currentRPM2; }
float WSS_GetRPM3() { return currentRPM3; }
float WSS_GetRPM4() { return currentRPM4; }

float WSS_GetSpeed1_MPH() { return currentSpeedMPS1 * MPS_TO_MPH; }
float WSS_GetSpeed2_MPH() { return currentSpeedMPS2 * MPS_TO_MPH; }
float WSS_GetSpeed3_MPH() { return currentSpeedMPS3 * MPS_TO_MPH; }
float WSS_GetSpeed4_MPH() { return currentSpeedMPS4 * MPS_TO_MPH; }
