// Anteater Electric Racing, 2026

#include "speaker.h"
#include "arduino_freertos.h"
#include <Arduino.h>

void Speaker_Init() { pinMode(speakerPin, arduino::INPUT); }

void Speaker_Play() {
    // Short descending fart sound
    for (int frequency = 180; frequency >= 65; frequency -= 12) {
        tone(speakerPin, frequency);
        vTaskDelay(pdMS_TO_TICKS(25));
    }

    noTone(speakerPin);
}
