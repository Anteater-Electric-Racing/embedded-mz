// Anteater Electric Racing, 2026

#include "speaker.h"
#include "arduino_freertos.h"
#include <Arduino.h>

void Speaker_Init() { pinMode(speakerPin, arduino::INPUT); }

void Speaker_Play() {
    tone(speakerPin, speakerFrequency);
    vTaskDelay(pdMS_TO_TICKS(speakerDuration));
    noTone(speakerPin);
}
