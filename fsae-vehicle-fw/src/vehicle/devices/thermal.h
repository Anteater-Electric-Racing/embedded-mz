
#include <stdint.h>
#include "vehicle/comms/telemetry.h"
#include "FreeRTOS.h"
#include "task.h"

constexpr int DUTY_CYCLE_MAX = 255;
constexpr int ANALOG_WRITE_FREQUENCY = 25000; // 25 kHz for Koolance
constexpr int FAN_WRITE_FREQ = 500;
constexpr int ANALOG_WRITE_RESOLUTION = 8; // 8-bit resolution (0-255)

constexpr int PUMP1_PIN = 29; // Define the PWM pin for the pump
constexpr int PUMP2_PIN = 28; // Define PWM pin for pump 2
constexpr int FAN_PIN = 7;

constexpr int PUMP_THRESHOLD = 45; // Temperature threshold in degrees Celsius
constexpr int FAN_THRESHOLD = 50;  // Temperature threshold in degrees Celsius
constexpr float MAX_OUTPUT = 100.0;

constexpr float PUMP1_PROPORIONAL_GAIN = 1.0; // tuning parameters for pump PID
constexpr float PUMP1_INTEGRAL_GAIN = 0.01;
constexpr float PUMP1_DERIVATIVE_GAIN = 0.1;

constexpr float PUMP2_PROPORTIONAL_GAIN = 1.0; // tuning parameters for pump 2 PID
constexpr float PUMP2_INTEGRAL_GAIN = 0.01;
constexpr float PUMP2_DERIVATIVE_GAIN = 0.1;

constexpr float FAN_PROPORIONAL_GAIN = 1.0; // tuning paramaters for fan PID
constexpr float FAN_INTEGRAL_GAIN = 0.01;
constexpr float FAN_DERIVATIVE_GAIN = 0.1;

void thermal_Init();

void thermal_regulate();

void thermal_forceOn();
void thermal_forceOff();

//PID controller state struct can be used for any PID controller with specified line removed
typedef struct {
    float integral = 0.0;
    float prevError = 0.0;
    float prevOutput = 0.0; //prevOutput is not necessary for a generic PID but is used here to prevent integral windup and can also be used to debug.
    TickType_t lastTime = 0;
    } PIDState;
// To use this PID controller for a different application, create a PID state and follow the comments
// explaining which lines are necissary for a general PID controller and which are specific to this application.
float computePID(PIDState state, float setPoint, float input, float propGain, float integralGain, float derivativeGain);

/*thermistor based update*/
void thermal_Update(uint32_t rawReading1, uint32_t rawReading2,
                    uint32_t rawReading3, uint32_t rawReading4);
