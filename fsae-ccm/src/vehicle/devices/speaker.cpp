// Anteater Electric Racing, 2026

#include "speaker.h"
#include "arduino_freertos.h"

void Speaker_Init() { pinMode(speakerPin, 1); }

void Speaker_Play() {
    tone(speakerPin, speakerFrequency);
    // digitalWrite(14, 0);
    //  vTaskDelay(pdMS_TO_TICKS(speakerDuration));
    //  noTone(speakerPin);
}
