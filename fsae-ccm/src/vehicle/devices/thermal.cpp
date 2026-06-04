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
    analogWrite(PUMP1_PIN, 0);
    analogWrite(PUMP2_PIN, 0);

    pinMode(FAN_PIN, OUTPUT);
    analogWriteFrequency(FAN_PIN,
                         FAN_WRITE_FREQ);           // 25 kHz for Koolance
    analogWriteResolution(ANALOG_WRITE_RESOLUTION); // 0-255
    analogWrite(FAN_PIN, 0);

    // thermal_forceOff();
}

/*open loop  control */
/**
 * faults to add: (temp out of bounds??)
 *
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
        static float temp = 80;
        float fanOutput = computePID(&fan, FAN_THRESHOLD, temp,
                             FAN_PROPORIONAL_GAIN,
                             FAN_INTEGRAL_GAIN,
                             FAN_DERIVATIVE_GAIN);
        float pump1Output = computePID(&pump1, FAN_THRESHOLD, temp,
                             PUMP1_PROPORIONAL_GAIN,
                             PUMP1_INTEGRAL_GAIN,
                             PUMP1_DERIVATIVE_GAIN);
        float pump2Ouput = computePID(&pump2, FAN_THRESHOLD, temp,
                             PUMP2_PROPORTIONAL_GAIN,
                             PUMP2_INTEGRAL_GAIN,
                             PUMP2_DERIVATIVE_GAIN);
        //max(DTI_GetDTIData()->controllerTemp, DTI_GetDTIData()->motorTemp);
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
       // analogWrite(FAN_PIN, DUTY_CYCLE_MAX * pidOutput); 

    Serial.println("temp:" + String(temp));
    Serial.println("Fan Output:" + String(fanOutput));
    Serial.println("Pump1 Output:" + String(pump1Output));
    Serial.println("Pump2 Output:" + String(pump2Ouput));
    temp -= (fanOutput + pump1Output + pump2Ouput)/3; // this is a very basic model of how the system responds to the outputs, just for testing PID
    if(temp <= 50){
        temp = 80;
    }
    fanOutput = 1 - fanOutput; // invert fan output because a higher output means we want to run the fan faster which means we want a lower duty cycle
    pump1Output = 1 - pump1Output; // invert pump output because a higher output means we want to run the pump faster which means we want a lower duty cycle
    pump2Ouput = 1 - pump2Ouput; // invert pump output because a higher output means we want to run the pump faster which means we want a lower duty cycle
}

float computePID(PIDState *state, float setPoint, float input, float propGain,
                 float integralGain, float derivativeGain) {
    TickType_t now = xTaskGetTickCount();
    float dt = (now - state->lastTime) / (double)configTICK_RATE_HZ; // Convert
    state->lastTime = now;
    float error = input - setPoint; // Find the size of error
    // From here to the end of if statement is not necissary for a general PID
    // controller
    if (input < setPoint) {
        state->integral = error * dt;
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
