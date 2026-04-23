#include <Arduino.h>
#include "thermal.h"
//#include "bus.h"

#define DUTY_CYCLE_MAX 255
#define ANALOG_WRITE_FREQUENCY 25000 // 25 kHz for Koolance
#define FAN_WRITE_FREQ 500
#define ANALOG_WRITE_RESOLUTION 8 // 8-bit resolution (0-255)

#define PUMP1_PIN 29 // Define the PWM pin for the pump
#define PUMP2_PIN 28 // Define PWM pin for pump 2
#define FAN_PIN 7

#define PUMP_THRESHOLD 45 // Temperature threshold in degrees Celsius
#define FAN_THRESHOLD 50  // Temperature threshold in degrees Celsius
#define MAX_OUTPUT 100.0

#define PUMP1_PROPORIONAL_GAIN 1.0 // tuning parameters for pump PID
#define PUMP1_INTEGRAL_GAIN 0.01
#define PUMP1_DERIVATIVE_GAIN 0.1

#define PUMP2_PROPORIONAL_GAIN 1.0 // tuning parameters for pump 2 PID
#define PUMP2_INTEGRAL_GAIN 0.01
#define PUMP2_DERIVATIVE_GAIN 0.1

#define FAN_PROPORIONAL_GAIN 1.0 // tuning paramaters for fan PID
#define FAN_INTEGRAL_GAIN 0.01
#define FAN_DERIVATIVE_GAIN 0.1

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

    pinMode(FAN_PIN, OUTPUT);
    analogWriteFrequency(FAN_PIN,
                         FAN_WRITE_FREQ);           // 25 kHz for Koolance
    analogWriteResolution(ANALOG_WRITE_RESOLUTION); // 0-255

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
void thermal_MCULoop() {
    static PIDState pump1{0.0, 0.0}, pump2{0.0,0.0}, fan{0.0,0.0};
    if(DTI_GetDTIData()->controllerTemp > DTI_GetDTIData() -> motorTemp){ 
        analogWrite(PUMP1_PIN, DUTY_CYCLE_MAX * computePID(&pump1, PUMP_THRESHOLD, DTI_GetDTIData()->controllerTemp, PUMP1_PROPORIONAL_GAIN, PUMP1_INTEGRAL_GAIN, PUMP1_DERIVATIVE_GAIN, 0.1)); 
        analogWrite(PUMP2_PIN, DUTY_CYCLE_MAX * computePID(&pump2, PUMP_THRESHOLD, DTI_GetDTIData()->controllerTemp, PUMP2_PROPORIONAL_GAIN, PUMP2_INTEGRAL_GAIN, PUMP2_DERIVATIVE_GAIN, 0.1));
        analogWrite(FAN_PIN, DUTY_CYCLE_MAX * computePID(&fan, FAN_THRESHOLD, DTI_GetDTIData()->controllerTemp, FAN_PROPORIONAL_GAIN, FAN_INTEGRAL_GAIN, FAN_DERIVATIVE_GAIN, 0.1));
    } else {
        analogWrite(PUMP1_PIN, DUTY_CYCLE_MAX * computePID(&pump1, PUMP_THRESHOLD, DTI_GetDTIData() -> motorTemp, PUMP1_PROPORIONAL_GAIN, PUMP1_INTEGRAL_GAIN, PUMP1_DERIVATIVE_GAIN, 0.1));
        analogWrite(PUMP2_PIN, DUTY_CYCLE_MAX * computePID(&pump2, PUMP_THRESHOLD, DTI_GetDTIData() -> motorTemp, PUMP2_PROPORIONAL_GAIN, PUMP2_INTEGRAL_GAIN, PUMP2_DERIVATIVE_GAIN, 0.1));
        analogWrite(FAN_PIN, DUTY_CYCLE_MAX * computePID(&fan, FAN_THRESHOLD, DTI_GetDTIData() -> motorTemp, FAN_PROPORIONAL_GAIN, FAN_INTEGRAL_GAIN, FAN_DERIVATIVE_GAIN, 0.1));
    }
}

double computePID(PIDState* state,double setPoint, double input, double propGain, double integralGain, double derivativeGain, double dt) {
    double error = input - setPoint; //Find the size of error
    if(input < setPoint){
        state->integral = error * dt;
            state->prevError = error;
            state->prevOutput = 0;
            return 0;
    }  
    if(state->prevOutput < 1.0 && state->prevOutput > 0.0) state->integral += error *dt;
    double derivative = (error - state ->prevError) / dt; // Calculate the derivative of the error (rate of change)
    double output = propGain * error + integralGain * state->integral + derivativeGain * derivative; // Compute the PID output using the proportional, integral, and derivative terms
    if(output > MAX_OUTPUT) output = MAX_OUTPUT;
    if(output < 0) output = 0; 
    output = output / MAX_OUTPUT; // scale output to be between 0 and 1 for PWM duty cycle
    state->prevOutput = output;
    state->prevError = error;
    return output; // Return the computed control output
}
