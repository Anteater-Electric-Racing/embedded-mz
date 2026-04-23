
#include <stdint.h>
#include "vehicle/comms/telemetry.h"

void thermal_Init();

void thermal_MCULoop();

void thermal_forceOn();
void thermal_forceOff();
typedef struct {
    double integral = 0.0;
    double prevError = 0.0;
    double prevOutput = 0.0;
    } PIDState;

double computePID(PIDState state, double setPoint, double input, double propGain, double integralGain, double derivativeGain, double dt);

/*thermistor based update*/
void thermal_Update(uint32_t rawReading1, uint32_t rawReading2,
                    uint32_t rawReading3, uint32_t rawReading4);
