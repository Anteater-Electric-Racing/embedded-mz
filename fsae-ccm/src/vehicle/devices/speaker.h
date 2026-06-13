// Anteater Electric Racing, 2026

#pragma once

#include <Arduino.h>

constexpr uint8_t speakerPin = 14; // old was 46
constexpr uint16_t speakerFrequency = 1300;
constexpr uint16_t speakerDuration = 3000;

void Speaker_Init();
void Speaker_Play();
