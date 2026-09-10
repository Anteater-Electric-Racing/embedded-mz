// Anteater Electric Racing, 2025

#include "gpio.h"
#include "utils.h"
#include <Arduino.h>

void gpioInit(void) {
    pinMode(SHUTDOWN_CTRL_PIN, OUTPUT);
    pinMode(FREQ_ACCU_PIN, INPUT_PULLDOWN);
    pinMode(FREQ_TS_PIN, INPUT_PULLDOWN);
    // pinMode(16, INPUT);
}
