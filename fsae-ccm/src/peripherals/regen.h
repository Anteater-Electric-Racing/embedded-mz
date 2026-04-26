#pragma once
#include <stdint.h>

struct float {
    float brakePercent; // 0 - 1 value
    float regenPercent; // 0 - 1 value
}
regenData;

float regen_GetRegenPercent();