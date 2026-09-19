// Anteater Electric Racing, 2026

#include "speaker.h"
#include "arduino_freertos.h"
#include <Arduino.h>

void Speaker_Init() { pinMode(speakerPin, arduino::OUTPUT); }

void Speaker_Play() {
        Serial.println("playin a sound type");
        vTaskDelay(pdMS_TO_TICKS(20));
}
