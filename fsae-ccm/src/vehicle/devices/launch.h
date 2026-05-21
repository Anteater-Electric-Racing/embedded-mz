#pragma once


constexpr float KP = 50.0f;
constexpr float KI = 5.0f;
constexpr float KD = 0.0f;
constexpr float KS = 0.0f;
constexpr float KV = 0.0f;
constexpr float MAX_TORQUE = 260.0f; // Max motor torque
constexpr float MIN_TORQUE = 0.0f;
constexpr float INTEGRAL_MAX = 20.0f;
constexpr float INTEGRAL_MIN = -20.0f;
constexpr float rpmConversion = 0.3f;


typedef enum {
    LAUNCH_STATE_OFF,
    LAUNCH_STATE_ON,
    LAUNCH_STATE_FAULT
} LaunchState;

void LaunchControl_Init();
float threadLaunchControl(void *pvParameters);
LaunchState Launch_getState();