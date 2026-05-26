#include "PID.h"

int pidConfig(PID *pid, float windupLimitMax, float windupLimitMin) {
    if (pid == nullptr) {
        return -1; // Error: Null pointer
    }
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->prevOutput = 0.0f;
    pid->lastTime = xTaskGetTickCount();
    pid->windUpLimitMax = windupLimitMax;
    pid->windUpLimitMin = windupLimitMin;
    return 0; // Success
}

int pidReset(PID *pid) {
    if (pid == nullptr) {
        return -1; // Error: Null pointer
    }
    pid->integral = 0.0f;
    pid->prevError = 0.0f;
    pid->prevOutput = 0.0f;
    pid->lastTime = xTaskGetTickCount();
    return 0; // Success
}

float computePID(PID *pid, float setPoint, float input, float propGain,
                 float integralGain, float derivativeGain) {
    TickType_t now = xTaskGetTickCount();
    float error = setPoint - input;
    float dt = (now - pid->lastTime) / (double)configTICK_RATE_HZ;
    pid->integral += error * dt;
    float derivative = (error - pid->prevError) / dt;
    if (pid->integral > pid->windUpLimitMax) {
        pid->integral = pid->windUpLimitMax;
    } else if (pid->integral < pid->windUpLimitMin) {
        pid->integral = pid->windUpLimitMin;
    }
    float output = propGain * error + integralGain * pid->integral +
                   derivativeGain * derivative;
    pid->prevError = error;
    pid->lastTime = now;
    pid->prevOutput = output;
    return output;
}