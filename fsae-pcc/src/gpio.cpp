// Anteater Electric Racing, 2025

#include "gpio.h"
#include "utils.h"
#include <Arduino.h>

void gpioInit(void) {
    pinMode(IR_PLUS, OUTPUT);
    pinMode(IR_MINUS, OUTPUT);
    pinMode(FREQ_ACCU_PIN, INPUT_PULLDOWN);
    pinMode(FREQ_TS_PIN, INPUT_PULLDOWN);
    // pinMode(16, INPUT);
}
