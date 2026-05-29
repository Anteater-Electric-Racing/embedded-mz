#ifndef PID_H
#define PID_H

#include "arduino_freertos.h"
// PID controller struct and functions, can be used for any PID controller but
// make sure to set limits for integral windup based on the setpoint you are
// trying to achieve.
typedef struct {
    float integral;
    float prevError;
    float prevOutput;
    TickType_t lastTime;
    float windUpLimitMax;
    float windUpLimitMin;
} PID;

int pidConfig(PID *pid, float windupLimitMax, float windupLimitMin);

int pidReset(PID *pid);

float computePID(PID *pid, float setPoint, float input, float propGain,
                 float integralGain, float derivativeGain);

#endif