// Anteater Electric Racing, 2026

#include "speaker.h"

void Speaker_Init() { pinMode(speakerPin, OUTPUT); }

void Speaker_Play() {
    tone(speakerPin, speakerFrequency);
    vTaskDelay(pdMS_TO_TICKS(speakerDuration));
    noTone(speakerPin);
}
