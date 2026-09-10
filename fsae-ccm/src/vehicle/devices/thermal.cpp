#include "thermal.h"
#include <Arduino.h>

#define OUTPUT 1

void thermal_forceOff() {
    analogWrite(PUMP1_PIN, 0);
    analogWrite(PUMP2_PIN, 0);
    analogWrite(FAN_PIN, DUTY_CYCLE_MAX);
}

void thermal_Init() {
    pinMode(PUMP1_PIN, OUTPUT);
    pinMode(PUMP2_PIN, OUTPUT);
    analogWriteFrequency(PUMP1_PIN,
                         ANALOG_WRITE_FREQUENCY);   // 25 kHz for Koolance
    analogWriteResolution(ANALOG_WRITE_RESOLUTION); // 0-255

    analogWriteFrequency(PUMP2_PIN,
                         ANALOG_WRITE_FREQUENCY);   // 25 kHz for Koolance
    analogWriteResolution(ANALOG_WRITE_RESOLUTION); // 0-255
    // analogWrite(PUMP1_PIN, 0);
    // analogWrite(PUMP2_PIN, 0);

    pinMode(FAN_PIN, OUTPUT);
    analogWriteFrequency(FAN_PIN,
                         FAN_WRITE_FREQ);           // 25 kHz for Koolance
    analogWriteResolution(ANALOG_WRITE_RESOLUTION); // 0-255
    // analogWrite(FAN_PIN, 0);

    // thermal_forceOff();
}

/*open loop  control */
/**
 * faults to add: (temp out of bounds??)
 */

void thermal_forceOn() {
    analogWrite(PUMP1_PIN, DUTY_CYCLE_MAX * 0.9);
    analogWrite(PUMP2_PIN, DUTY_CYCLE_MAX * 0.9);
    analogWrite(FAN_PIN, DUTY_CYCLE_MAX * 0.1);
}

/* Recieves the temperature of both  */
void thermal_regulate() {
    static PIDState pump1{0.0, 0.0, 0.0, xTaskGetTickCount()},
        pump2{0.0, 0.0, 0.0, xTaskGetTickCount()},
        fan{0.0, 0.0, 0.0, xTaskGetTickCount()};
    float temp =
        max(DTI_GetDTIData()->controllerTemp, DTI_GetDTIData()->motorTemp);
    float fanOutput =
        computePID(&fan, FAN_THRESHOLD, temp, FAN_PROPORIONAL_GAIN,
                   FAN_INTEGRAL_GAIN, FAN_DERIVATIVE_GAIN);
    float pump1Output =
        computePID(&pump1, FAN_THRESHOLD, temp, PUMP1_PROPORIONAL_GAIN,
                   PUMP1_INTEGRAL_GAIN, PUMP1_DERIVATIVE_GAIN);
    float pump2Output =
        computePID(&pump2, FAN_THRESHOLD, temp, PUMP2_PROPORTIONAL_GAIN,
                   PUMP2_INTEGRAL_GAIN, PUMP2_DERIVATIVE_GAIN);
    //

    /*analogWrite(PUMP1_PIN,
                DUTY_CYCLE_MAX * computePID(&pump1, PUMP_THRESHOLD, temp,
                                            PUMP1_PROPORIONAL_GAIN,
                                            PUMP1_INTEGRAL_GAIN,
                                            PUMP1_DERIVATIVE_GAIN));*/
    /*analogWrite(PUMP2_PIN,
                DUTY_CYCLE_MAX * computePID(&pump2, PUMP_THRESHOLD, temp,
                                            PUMP2_PROPORTIONAL_GAIN,
                                            PUMP2_INTEGRAL_GAIN,
                                            PUMP2_DERIVATIVE_GAIN)); */
    analogWrite(PUMP1_PIN, DUTY_CYCLE_MAX * (1 - pump1Output));
    analogWrite(PUMP2_PIN, DUTY_CYCLE_MAX * (1 - pump2Output));
    analogWrite(FAN_PIN, DUTY_CYCLE_MAX * (1 - fanOutput));
}

float computePID(PIDState *state, float setPoint, float input, float propGain,
                 float integralGain, float derivativeGain) {
    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - state->lastTime;

    if (elapsed == 0) {
        return state->prevOutput;
    }
    float dt = static_cast<float>(elapsed) /
               static_cast<float>(configTICK_RATE_HZ); // Convert
    state->lastTime = now;
    float error = input - setPoint; // Find the size of error
    // From here to the end of if statement is not necissary for a general PID
    // controller
    if (input < setPoint) {
        state->integral = 0.0f; // Was: error * dt
        state->prevError = error;
        state->prevOutput = 0;
        return 0;
    }
    if (state->prevOutput < 1.0 && state->prevOutput > 0.0)
        state->integral +=
            error * dt; // disregard for normal applications of PID
    float derivative =
        (error - state->prevError) /
        dt; // Calculate the derivative of the error (rate of change)
    float output =
        propGain * error + integralGain * state->integral +
        derivativeGain *
            derivative; // Compute the PID output using the proportional,
                        // integral, and derivative terms
    // From here to output = output / MAX_OUTPUT is not necissary for a general
    // PID controller
    if (output > MAX_OUTPUT)
        output = MAX_OUTPUT;
    if (output < 0)
        output = 0;
    output =
        output /
        MAX_OUTPUT; // scale output to be between 0 and 1 for PWM duty cycle
    state->prevOutput = output;
    state->prevError = error;
    return output; // Return the computed control output
}
